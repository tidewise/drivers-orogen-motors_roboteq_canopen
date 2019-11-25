/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */


#include "Task.hpp"
#include <base-logging/Logging.hpp>

using namespace base;
using namespace std;
using namespace motors_roboteq_canopen;

Task::Task(string const& name)
    : TaskBase(name)
{
    _state_change_timeout.set(base::Time::fromSeconds(1));
}

Task::~Task()
{
}

bool Task::configureHook()
{
    delete m_driver;
    m_driver = nullptr;
    delete m_state_machine;
    m_state_machine = nullptr;

    auto channel_configurations = _channel_configurations.get();
    m_channel_count = channel_configurations.size();
    if (m_channel_count == 0) {
        LOG_ERROR_S << "no channels configured" << std::endl;
        return false;
    }
    else if (m_channel_count > Driver::MAX_CHANNEL_COUNT) {
        LOG_ERROR_S << "driver supports only up to "
                    << Driver::MAX_CHANNEL_COUNT << " channels" << std::endl;
        return false;
    }

    m_state_machine = new canopen_master::StateMachine(_node_id.get());
    m_driver = new Driver(*m_state_machine, channel_configurations.size());
    m_slave = m_driver;
    m_joint_state.elements.resize(m_channel_count);
    m_channel_ignored.resize(m_channel_count);
    for (int i = 0; i < m_channel_count; ++i) {
        m_channel_ignored[i] =
            (channel_configurations[i].operation_mode == OPERATION_MODE_NONE);
    }

    if (! TaskBase::configureHook()) {
        return false;
    }

    // The Roboteq firmware does not send the Boot-up message on RESET,
    // so we can't guard this state transition
    //
    // It seems to also require a ENTER_PRE_OPERATIONAL in sonme conditions
    // (namely, if in STOPPED state). WE make one mandatory, in any case it
    // will just be ignored and we'll be in PRE_OPERATIONAL state
    _can_out.write(m_slave->queryNodeStateTransition(canopen_master::NODE_RESET));
    usleep(100000);
    toNMTState(canopen_master::NODE_PRE_OPERATIONAL,
               canopen_master::NODE_ENTER_PRE_OPERATIONAL,
               base::Time::fromMilliseconds(100));
    channelsToSwitchOn();

    for (int i = 0; i < m_channel_count; ++i) {
        Channel& channel = m_driver->getChannel(i);
        ChannelConfiguration const& config = channel_configurations[i];

        writeSDOs(channel.queryOperationModeDownload(config.operation_mode));
        channel.setOperationMode(config.operation_mode);
        if (config.operation_mode == OPERATION_MODE_NONE) {
            continue;
        }

        channel.setFactors(config.factors);
    }

    toNMTState(canopen_master::NODE_OPERATIONAL,
               canopen_master::NODE_START,
               base::Time::fromMilliseconds(100));

    vector<canbus::Message> tpdo_setup;
    m_driver->setupJointStateTPDOs(tpdo_setup, 0, _joint_state_settings.get());
    writeSDOs(tpdo_setup);
    vector<canbus::Message> rpdo_setup;
    m_driver->setupJointCommandRPDOs(rpdo_setup, 0, _joint_command_settings.get());
    writeSDOs(rpdo_setup);

    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook()) {
        return false;
    }

    for (int i = 0; i < m_channel_count; ++i) {
        if (m_channel_ignored[i]) {
            continue;
        }

        Channel& channel = m_driver->getChannel(i);
        writeSDOs(channel.sendDS402Transition(ControlWord::ENABLE_OPERATION, false));
    }
    return true;
}

void Task::updateHook()
{
    canbus::Message msg;
    while (_can_in.read(msg, false) == RTT::NewData) {
        m_driver->process(msg);
    }

    base::samples::Joints command;
    while (_joint_cmd.read(command, false) == RTT::NewData) {
        m_driver->setJointCommand(command);
        auto const& messages = m_driver->getRPDOMessages();
        for (auto const& msg : messages) {
            _can_out.write(msg);
        }
    }

    bool has_update = true;
    for (int i = 0; i < m_channel_count; ++i) {
        auto& channel = m_driver->getChannel(i);
        if (!channel.hasJointStateUpdate()) {
            has_update = false;
            break;
        }

        m_joint_state.elements[i] = channel.getJointState();
    }

    if (has_update) {
        _joint_samples.write(m_joint_state);
        for (int i = 0; i < m_channel_count; ++i) {
            m_driver->getChannel(i).resetJointStateTracking();
        }
    }

    TaskBase::updateHook();
}

void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    for (int i = 0; i < m_channel_count; ++i) {
        if (m_channel_ignored[i]) {
            continue;
        }

        Channel& channel = m_driver->getChannel(i);
        writeSDOs(channel.sendDS402Transition(ControlWord::DISABLE_OPERATION, false));
    }

    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    channelsToSwitchOnDisabled();
    TaskBase::cleanupHook();
}

void Task::waitDS402State(Channel& channel, StatusWord::State state) {
    base::Time deadline = base::Time::now() + _state_change_timeout.get();
    while (true) {
        readSDOs(channel.queryDS402Status());
        StatusWord current = channel.getDS402Status();
        if (current.state == state) {
            break;
        }
        else if (base::Time::now() > deadline) {
            throw std::runtime_error("timed out while waiting for "\
                                     "transition to state " + to_string(state));
        }
        usleep(10000);
    }
}

void Task::channelsToSwitchOnDisabled() {
    for (int i = 0; i < m_channel_count; ++i) {
        if (m_channel_ignored[i]) {
            continue;
        }

        Channel& channel = m_driver->getChannel(i);
        readSDOs(channel.queryDS402Status());
        StatusWord current = channel.getDS402Status();

        switch (current.state) {
            case StatusWord::FAULT_REACTION_ACTIVE:
                waitDS402State(channel, StatusWord::FAULT);
            case StatusWord::FAULT:
                writeSDOs(channel.sendDS402Transition(ControlWord::FAULT_RESET, false));
                break;
            case StatusWord::SWITCH_ON_DISABLED:
                break;
            case StatusWord::NOT_READY_TO_SWITCH_ON:
            case StatusWord::READY_TO_SWITCH_ON:
            case StatusWord::SWITCH_ON:
            case StatusWord::OPERATION_ENABLED:
                writeSDOs(channel.
                          sendDS402Transition(ControlWord::DISABLE_VOLTAGE, false));
                break;
            case StatusWord::QUICK_STOP_ACTIVE:
                waitDS402State(channel, StatusWord::SWITCH_ON_DISABLED);
                break;
        }
    }
}

void Task::channelsToSwitchOn() {
    for (int i = 0; i < m_channel_count; ++i) {
        if (m_channel_ignored[i]) {
            continue;
        }

        Channel& channel = m_driver->getChannel(i);
        readSDOs(channel.queryDS402Status());
        StatusWord current = channel.getDS402Status();

        switch (current.state) {
            case StatusWord::FAULT_REACTION_ACTIVE:
                waitDS402State(channel, StatusWord::FAULT);
            case StatusWord::FAULT:
                writeSDOs(channel.sendDS402Transition(ControlWord::FAULT_RESET, false));
            case StatusWord::NOT_READY_TO_SWITCH_ON:
            case StatusWord::SWITCH_ON_DISABLED:
                writeSDOs(channel.sendDS402Transition(ControlWord::SHUTDOWN, false));
            case StatusWord::READY_TO_SWITCH_ON:
                writeSDOs(channel.sendDS402Transition(ControlWord::SWITCH_ON, false));
                break;
            case StatusWord::SWITCH_ON:
                break;
            case StatusWord::OPERATION_ENABLED:
                writeSDOs(channel.sendDS402Transition(ControlWord::DISABLE_OPERATION, false));
                break;
            case StatusWord::QUICK_STOP_ACTIVE:
                waitDS402State(channel, StatusWord::SWITCH_ON_DISABLED);
                writeSDOs(channel.sendDS402Transition(ControlWord::SHUTDOWN, false));
                writeSDOs(channel.sendDS402Transition(ControlWord::SWITCH_ON, false));
                break;
        }
    }
}