/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include "Helpers.hpp"
#include <base-logging/Logging.hpp>

#include <regex>

using namespace std;
using namespace motors_roboteq_canopen;

Task::Task(std::string const& name)
    : TaskBase(name)
{
    _status_query_period.set(base::Time::fromSeconds(5));
    _feedback_timeout.set(base::Time::fromSeconds(1));
    _input_timeout.set(base::Time::fromSeconds(1));
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
    size_t channel_count = channel_configurations.size();
    if (channel_count == 0) {
        LOG_ERROR_S << "no channels configured" << std::endl;
        return false;
    }

    m_input_timeout = _input_timeout.get();

    m_state_machine = new canopen_master::StateMachine(_node_id.get());
    m_driver = new Driver(*m_state_machine, channel_configurations.size());
    m_slave = m_driver;
    m_joint_state.elements.resize(channel_count);

    if (!TaskBase::configureHook()) {
        return false;
    }

    m_feedback_timeout = _feedback_timeout.get();

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

    auto analog_input_conf = _analog_input_configuration.get();
    for (auto conf : analog_input_conf) {
        if (conf.converted) {
            m_driver->setConvertedAnalogInputEnableInTPDO(conf.index, true);
        }
        else {
            m_driver->setAnalogInputEnableInTPDO(conf.index, true);
        }
    }
    m_analog_inputs.resize(analog_input_conf.size());

    vector<canbus::Message> tpdo_setup;
    int pdoIndex =
        m_driver->setupJointStateTPDOs(tpdo_setup, 0, _joint_state_settings.get());
    pdoIndex =
        m_driver->setupAnalogTPDOs(tpdo_setup, pdoIndex, _analog_input_settings.get());
    if (_status_use_pdo.get()) {
        m_driver->setupStatusTPDOs(tpdo_setup, pdoIndex, _status_settings.get());
    }
    writeSDOs(tpdo_setup);

    m_edge_triggered_digital_output = _edge_triggered_digital_output.get();
    m_default_digital_output = _digital_output_config.get();
    m_managed_digital_outputs =
        helpers::parseManagedDigitalOutputs(m_default_digital_output.gpio_paths);
    m_managed_digital_output_mask =
        helpers::managedDigitalOutputMask(m_managed_digital_outputs);
    m_raw_default_digital_output =
        helpers::commandToRaw(m_default_digital_output.defaults,
            m_managed_digital_outputs);

    writeDefaultDigitalOutput(true);
    _digital_output.write({base::Time::now(),
        m_driver->parseDigitalOutput(m_raw_default_digital_output,
            m_managed_digital_outputs)});

    return true;
}
bool Task::startHook()
{
    if (!TaskBase::startHook()) {
        return false;
    }

    m_status_query_deadline = base::Time();
    m_feedback_deadline = base::Time::now() + m_feedback_timeout;
    m_input_deadline = base::Time::now() + m_input_timeout;
    m_digital_cmd_deadline = base::Time::now() + m_default_digital_output.timeout;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();

    if (base::Time::now() > m_feedback_deadline) {
        return exception(FEEDBACK_TIMEOUT);
    }

    if (base::Time::now() > m_input_deadline && state() != INPUT_TIMEOUT) {
        writeSDOs(m_driver->queryMotorStop());
        state(INPUT_TIMEOUT);
    }

    if (base::Time::now() > m_digital_cmd_deadline) {
        writeDefaultDigitalOutput();
    }

    handleDigitalCommand();

    canbus::Message msg;
    while (_can_in.read(msg, false) == RTT::NewData) {
        m_driver->process(msg);
    }

    if (m_driver->hasAnalogInputUpdate() && m_driver->hasConvertedAnalogInputUpdate()) {
        outputAnalog();
    }

    if (_status_use_pdo.get()) {
        writeStatusPort();
    }
    else {
        handleStatusQuery();
    }

    outputDigital();

    bool has_update = true;
    for (size_t i = 0; i < m_driver->getChannelCount(); ++i) {
        auto& channel = m_driver->getChannel(i);
        if (!channel.hasJointStateUpdate()) {
            has_update = false;
            break;
        }
        m_feedback_deadline = base::Time::now() + m_feedback_timeout;

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

        m_input_deadline = base::Time::now() + m_input_timeout;
        if (state() == INPUT_TIMEOUT) {
            state(RUNNING);
        }
    }
}

void Task::handleStatusQuery()
{
    if (m_status_sdos.empty()) {
        if (m_status_query_deadline > base::Time::now()) {
            return;
        }

        m_status_sdos = m_driver->queryControllerStatus();
        m_status_sdos.push_back(m_driver->queryReadDigitalOutput());
        m_status_query_deadline = base::Time::now() + _status_query_period.get();
    }

    auto msg = m_status_sdos.back();
    m_status_sdos.pop_back();
    readSDO(msg);

    if (m_status_sdos.empty()) {
        writeStatusPort();
    }
}

void Task::writeStatusPort()
{
    try {
        _controller_status.write(m_driver->getControllerStatus());
    }
    catch (canopen_master::ObjectNotRead&) {
    }
}

void Task::outputAnalog()
{
    base::Time now = base::Time::now();

    auto const& conf = _analog_input_configuration.get();
    for (size_t i = 0; i < conf.size(); ++i) {
        m_analog_inputs[i].time = now;

        auto const& input = conf[i];
        if (input.converted) {
            m_analog_inputs[i].data =
                m_driver->get<ConvertedAnalogInput>(0, input.index + 1);
        }
        else {
            m_analog_inputs[i].data = m_driver->get<AnalogInput>(0, input.index + 1);
        }
    }
    _analog_inputs.write(m_analog_inputs);

    m_driver->resetAnalogInputTracking();
    m_driver->resetConvertedAnalogInputTracking();
}

bool Task::handleDigitalCommand()
{
    linux_gpios::GPIOState digital_cmd;
    if (_digital_cmd.read(digital_cmd) != RTT::NewData) {
        return false;
    }

    if (digital_cmd.states.size() != m_managed_digital_outputs.size()) {
        throw std::runtime_error(
            "Digital output command size and managed output sizes mismatch");
    }

    auto updated = helpers::difference(digital_cmd.states,
        m_driver->readDigitalOutput(m_managed_digital_outputs));
    std::vector<canbus::Message> messages;
    messages.reserve(updated.size());
    for (std::uint8_t output : updated) {
        messages.push_back(
            m_driver->queryWriteDigitalOutput(m_managed_digital_outputs[output],
                digital_cmd.states[output].data));
    }
    writeSDOs(messages);

    m_digital_cmd_deadline = base::Time::now() + m_default_digital_output.timeout;
    return true;
}

void Task::writeDefaultDigitalOutput(bool force)
{
    std::uint16_t current_output =
        m_driver->readDigitalOutputRaw() & m_managed_digital_output_mask;
    if (force || current_output == m_raw_default_digital_output) {
        return;
    }

    const std::size_t n = m_managed_digital_outputs.size();

    std::vector<canbus::Message> messages;
    messages.reserve(n);
    for (std::size_t i = 0; i < n; i++) {
        messages.push_back(m_driver->queryWriteDigitalOutput(m_managed_digital_outputs[i],
            (bool)m_default_digital_output.defaults[i]));
    }
    writeSDOs(messages);
}

void Task::outputDigital()
{
    std::uint16_t current_output =
        m_driver->readDigitalOutputRaw() & m_managed_digital_output_mask;

    bool should_not_output =
        m_edge_triggered_digital_output &&
        current_output == m_last_processed_digital_output_raw_reading;

    if (should_not_output) {
        return;
    }

    m_last_processed_digital_output_raw_reading = current_output;

    _digital_output.write(
        {base::Time::now(), m_driver->readDigitalOutput(m_managed_digital_outputs)});
}

void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    readSDOs(m_driver->queryControllerStatus());
    writeStatusPort();
    writeSDOs(m_driver->queryMotorStop());
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}
