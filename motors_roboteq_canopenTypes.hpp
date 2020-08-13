#ifndef motors_roboteq_canopen_TYPES_HPP
#define motors_roboteq_canopen_TYPES_HPP

#include <motors_roboteq_canopen/Objects.hpp>
#include <motors_roboteq_canopen/Factors.hpp>

namespace motors_roboteq_canopen {
    /** Configuration for a single channel
     *
     * Channels with operation_mode set to OPERATION_MODE_NONE are ignored
     */
    struct ChannelConfiguration {
        ControlModes control_mode = CONTROL_NONE;
        Factors factors;
    };

    /** Configuration for a single channel
     *
     * Channels with operation_mode set to OPERATION_MODE_NONE are ignored
     */
    struct DS402ChannelConfiguration {
        DS402OperationModes operation_mode = DS402_OPERATION_MODE_NONE;
        Factors factors;
    };

    /** Configuration of the analog inputs to be read */
    struct AnalogInputConfiguration {
        /** The analog input index (base zero) */
        int index = 0;

        /** Whether we should read the raw or converted value */
        bool converted = true;
    };
}

#endif

