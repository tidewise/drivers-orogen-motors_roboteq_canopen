#include "Helpers.hpp"

std::vector<uint8_t> motors_roboteq_canopen::helpers::parseManagedDigitalOutputs(
    std::vector<std::string> const& outputs)
{
    std::vector<uint8_t> output_id;
    output_id.reserve(outputs.size());

    for (std::string const& id_string : outputs) {
        std::int32_t out = std::stoi(id_string);
        if (out < 1 || out > 16) {
            throw std::runtime_error(
                "Managed digital output id" + id_string + "is not within [1,16] range");
        }
        output_id.push_back(out);
    }

    return output_id;
}

std::vector<uint8_t> motors_roboteq_canopen::helpers::difference(
    std::vector<raw_io::Digital> const& s1,
    std::vector<raw_io::Digital> const& s2)
{
    std::vector<uint8_t> diff;
    diff.reserve(s1.size());

    for (std::size_t i = 0; i < s1.size(); i++) {
        if (s1[i].data != s2[i].data) {
            diff.push_back(i);
        }
    }
    return diff;
}

std::uint16_t motors_roboteq_canopen::helpers::managedDigitalOutputMask(
    std::vector<std::uint8_t> const& managed_digital_outputs)
{
    std::uint16_t mask{0};
    for (std::uint8_t output : managed_digital_outputs) {
        std::uint8_t index = output - 1;
        mask |= (1 << index);
    }
    return mask;
}

std::uint16_t motors_roboteq_canopen::helpers::commandToRaw(
    std::vector<std::uint8_t> const& cmd,
    std::vector<std::uint8_t> const& managaged_digital_outputs)
{
    std::uint16_t raw{0};
    for (std::size_t i = 0; i < cmd.size(); i++) {
        if (cmd[i]) {
            raw |= 1 << (managaged_digital_outputs[i] - 1);
        }
    }
    return raw;
}