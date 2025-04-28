#ifndef MOTORS_ROBOTEQ_CANOPEN_TASK_HELPERS_HPP
#define MOTORS_ROBOTEQ_CANOPEN_TASK_HELPERS_HPP

#include <raw_io/Digital.hpp>
#include <vector>
#include <string>

namespace motors_roboteq_canopen {
    namespace helpers {
        /**
         * Converts digital outputs id string configuration to integers
         */
        std::vector<uint8_t> parseManagedDigitalOutputs(
            std::vector<std::string> const& outputs);

        /**
         * Returns the indexes of mismatching data values
         */
        std::vector<uint8_t> difference(std::vector<raw_io::Digital> const& s1,
            std::vector<raw_io::Digital> const& s2);

        /**
         *  Computes the managed output mask
         *
         * @param managed_digital_outputs are the managed outputs numbers, values range
         * from 1 to MAX, where MAX is the controller digital output size
         */
        std::uint16_t managedDigitalOutputMask(
            std::vector<std::uint8_t> const& managed_digital_outputs);

        /**
         * Computes the corresponding raw reading for a command
         * @param cmd [0, 1] sequence
         * @param managed_digital_outputs are the managed outputs numbers, values range
         * from 1 to MAX, where MAX is the controller digital output size
         */
        std::uint16_t commandToRaw(std::vector<std::uint8_t> const& cmd,
            std::vector<std::uint8_t> const& managaged_digital_outputs);

    }
}

#endif