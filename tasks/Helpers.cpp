#include "Helpers.hpp"

using namespace std;

vector<uint8_t> motors_roboteq_canopen::helpers::parseManagedDigitalOutputs(
    vector<string> const& outputs)
{
    vector<uint8_t> output_id;
    output_id.reserve(outputs.size());

    for (string const& id_string : outputs) {
        int32_t out = stoi(id_string);
        if (out < 1 || out > 16) {
            throw runtime_error(
                "Managed digital output id" + id_string + "is not within [1,16] range");
        }
        output_id.push_back(out);
    }

    return output_id;
}

vector<uint8_t> motors_roboteq_canopen::helpers::difference(
    vector<raw_io::Digital> const& s1,
    vector<raw_io::Digital> const& s2)
{
    vector<uint8_t> diff;
    diff.reserve(s1.size());

    for (size_t i = 0; i < s1.size(); i++) {
        if (s1[i].data != s2[i].data) {
            diff.push_back(i);
        }
    }
    return diff;
}

uint16_t motors_roboteq_canopen::helpers::managedDigitalOutputMask(
    vector<uint8_t> const& managed_digital_outputs)
{
    uint16_t mask{0};
    for (uint8_t output : managed_digital_outputs) {
        uint8_t index = output - 1;
        mask |= (1 << index);
    }
    return mask;
}

uint16_t motors_roboteq_canopen::helpers::commandToRaw(
    vector<uint8_t> const& cmd,
    vector<uint8_t> const& managed_digital_outputs)
{
    uint16_t raw{0};
    for (size_t i = 0; i < cmd.size(); i++) {
        if (cmd[i]) {
            raw |= 1 << (managed_digital_outputs[i] - 1);
        }
    }
    return raw;
}
