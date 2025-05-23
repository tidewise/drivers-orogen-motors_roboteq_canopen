/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "DigitalOutMultiplexerTask.hpp"
#include "base-logging/Logging.hpp"
#include "base/Time.hpp"
#include "linux_gpios/linux_gpiosTypes.hpp"

using namespace linux_gpios;
using namespace motors_roboteq_canopen;
using namespace std;

DigitalOutMultiplexerTask::DigitalOutMultiplexerTask(std::string const& name)
    : DigitalOutMultiplexerTaskBase(name)
{
}

DigitalOutMultiplexerTask::~DigitalOutMultiplexerTask()
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See DigitalOutMultiplexerTask.hpp for more detailed
// documentation about them.

bool DigitalOutMultiplexerTask::configureHook()
{
    if (!DigitalOutMultiplexerTaskBase::configureHook())
        return false;
    return true;
}

bool DigitalOutMultiplexerTask::startHook()
{
    if (!DigitalOutMultiplexerTaskBase::startHook())
        return false;
    return true;
}

void DigitalOutMultiplexerTask::updateHook()
{
    DigitalOutMultiplexerTaskBase::updateHook();

    GPIOState overall_command;
    overall_command.states.resize(2);

    GPIOState clutch_cmd;
    if (_clutch_command.read(clutch_cmd) == RTT::NewData) {
        if (clutch_cmd.states.size() != 1) {
            throw runtime_error("clutch command must have a single state, but it has " +
                                clutch_cmd.states.size());
        }

        overall_command.states[0] = clutch_cmd.states.front();
    }
    GPIOState controller_selection_cmd;
    if (_controller_selection_command.read(controller_selection_cmd) == RTT::NewData) {
        if (controller_selection_cmd.states.size() != 1) {
            throw runtime_error(
                "controller selection command must have a single state, but it has " +
                controller_selection_cmd.states.size());
        }

        overall_command.states[1] = controller_selection_cmd.states.front();
    }

    overall_command.time = base::Time::now();
    _digital_out_command.write(overall_command);
}

void DigitalOutMultiplexerTask::errorHook()
{
    DigitalOutMultiplexerTaskBase::errorHook();
}

void DigitalOutMultiplexerTask::stopHook()
{
    DigitalOutMultiplexerTaskBase::stopHook();
}

void DigitalOutMultiplexerTask::cleanupHook()
{
    DigitalOutMultiplexerTaskBase::cleanupHook();
}
