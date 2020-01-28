/* Generated from orogen/lib/orogen/templates/tasks/DS402Task.cpp */


#include "DS402Task.hpp"
#include <base-logging/Logging.hpp>

using namespace base;
using namespace std;
using namespace motors_roboteq_canopen;

DS402Task::DS402Task(string const& name)
    : DS402TaskBase(name)
{
    _state_change_timeout.set(base::Time::fromSeconds(1));
}

DS402Task::~DS402Task()
{
}

bool DS402Task::configureHook()
{
    delete m_driver;
    m_driver = nullptr;
    delete m_state_machine;
    m_state_machine = nullptr;

    auto channel_configurations = _channel_configurations.get();
    size_t channel_count = channel_configurations.size();
    if (channel_count == 0) {
        LOG_ERROR_S << "no channels configured" << std::endl;
        return false;
    }

    m_state_machine = new canopen_master::StateMachine(_node_id.get());
    m_driver = new DS402Driver(*m_state_machine, channel_configurations.size());
    m_slave = m_driver;
    m_joint_state.elements.resize(channel_count);

    if (! DS402TaskBase::configureHook()) {
        return false;
    }

    // The Roboteq firmware does not send the Boot-up message on RESET,
    // so we can't guard this state transition
    //
    // It seems to also require a ENTER_PRE_OPERATIONAL in sonme conditions
    // (namely, if in STOPPED state). WE make one mandatory, in any case it
    // will just be ignored and we'll be in PRE_OPERATIONAL state
    _can_out.write(m_slave->queryNodeStateTransition(canopen_master::NODE_RESET));
    usleep(1000000);
    toNMTState(canopen_master::NODE_PRE_OPERATIONAL,
               canopen_master::NODE_ENTER_PRE_OPERATIONAL,
               base::Time::fromMilliseconds(100));

    toNMTState(canopen_master::NODE_OPERATIONAL,
               canopen_master::NODE_START,
               base::Time::fromMilliseconds(100));

    channelsToSwitchOn();
    for (size_t i = 0; i < channel_count; ++i) {
        DS402Channel& channel = m_driver->getChannel(i);
        DS402ChannelConfiguration const& config = channel_configurations[i];

        writeSDOs(channel.queryOperationModeDownload(config.operation_mode));
        channel.setOperationMode(config.operation_mode);
        if (config.operation_mode == DS402_OPERATION_MODE_NONE) {
            continue;
        }

        channel.setFactors(config.factors);
    }

    vector<canbus::Message> tpdo_setup;
    m_driver->setupJointStateTPDOs(tpdo_setup, 0, _joint_state_settings.get());
    writeSDOs(tpdo_setup);

    return true;
}
bool DS402Task::startHook()
{
    if (! DS402TaskBase::startHook()) {
        return false;
    }

    for (size_t i = 0; i < m_driver->getChannelCount(); ++i) {
        DS402Channel& channel = m_driver->getChannel(i);
        if (channel.isIgnored()) {
            continue;
        }
        writeSDOs(channel.sendDS402Transition(ControlWord::ENABLE_OPERATION, false));
    }
    return true;
}

void DS402Task::updateHook()
{
    canbus::Message msg;
    while (_can_in.read(msg, false) == RTT::NewData) {
        m_driver->process(msg);
    }

    bool has_update = true;
    for (size_t i = 0; i < m_driver->getChannelCount(); ++i) {
        auto& channel = m_driver->getChannel(i);
        if (!channel.hasJointStateUpdate()) {
            has_update = false;
            break;
        }

        m_joint_state.elements[i] = channel.getJointState();
    }

    if (!has_update) {
        return;
    }

    m_joint_state.time = base::Time::now();
    _joint_samples.write(m_joint_state);
    for (size_t i = 0; i < m_driver->getChannelCount(); ++i) {
        m_driver->getChannel(i).resetJointStateTracking();
    }

    base::samples::Joints command;
    if (_joint_cmd.read(command, false) == RTT::NewData) {
        m_driver->setJointCommand(command);
        auto const& messages = m_driver->queryJointCommandDownload();
        writeSDOs(messages);
    }

    DS402TaskBase::updateHook();
}

void DS402Task::errorHook()
{
    DS402TaskBase::errorHook();
}
void DS402Task::stopHook()
{
    for (size_t i = 0; i < m_driver->getChannelCount(); ++i) {
        DS402Channel& channel = m_driver->getChannel(i);
        if (channel.isIgnored()) {
            continue;
        }
        writeSDOs(channel.sendDS402Transition(ControlWord::DISABLE_OPERATION, false));
    }

    DS402TaskBase::stopHook();
}
void DS402Task::cleanupHook()
{
    channelsToSwitchOnDisabled();
    DS402TaskBase::cleanupHook();
}

void DS402Task::waitDS402State(DS402Channel& channel, StatusWord::State state) {
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

void DS402Task::channelsToSwitchOnDisabled() {
    for (size_t i = 0; i < m_driver->getChannelCount(); ++i) {
        DS402Channel& channel = m_driver->getChannel(i);
        if (channel.isIgnored()) {
            continue;
        }

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

void DS402Task::channelsToSwitchOn() {
    for (size_t i = 0; i < m_driver->getChannelCount(); ++i) {
        DS402Channel& channel = m_driver->getChannel(i);
        if (channel.isIgnored()) {
            continue;
        }

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