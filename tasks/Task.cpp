/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <base-logging/Logging.hpp>

using namespace std;
using namespace motors_roboteq_canopen;

Task::Task(std::string const& name)
    : TaskBase(name) {
}

Task::~Task() {
}

bool Task::configureHook() {
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
    m_driver = new Driver(*m_state_machine, channel_configurations.size());
    m_slave = m_driver;
    m_joint_state.elements.resize(channel_count);

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
    usleep(1000000);
    toNMTState(canopen_master::NODE_PRE_OPERATIONAL,
               canopen_master::NODE_ENTER_PRE_OPERATIONAL,
               base::Time::fromMilliseconds(100));

    toNMTState(canopen_master::NODE_OPERATIONAL,
               canopen_master::NODE_START,
               base::Time::fromMilliseconds(100));

    writeSDOs(m_driver->queryMotorStop());

    for (size_t i = 0; i < channel_count; ++i) {
        Channel& channel = m_driver->getChannel(i);
        ChannelConfiguration const& config = channel_configurations[i];
        channel.setControlMode(config.control_mode);
        channel.setFactors(config.factors);
    }

    vector<canbus::Message> tpdo_setup;
    m_driver->setupJointStateTPDOs(tpdo_setup, 0, _joint_state_settings.get());
    writeSDOs(tpdo_setup);

    return true;

}
bool Task::startHook()
{
    if (! TaskBase::startHook()) {
        return false;
    }
    return true;
}
void Task::updateHook()
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

    TaskBase::updateHook();
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    writeSDOs(m_driver->queryMotorStop());
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}
