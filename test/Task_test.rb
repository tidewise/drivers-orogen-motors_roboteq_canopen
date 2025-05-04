# frozen_string_literal: true

using_task_library "motors_roboteq_canopen"

require "canopen_master/test_helpers"

describe OroGen.motors_roboteq_canopen.Task do
    run_live

    include CANOpen::TestHelpers

    attr_reader :task
    before do
        @task = syskit_deploy(
            OroGen.motors_roboteq_canopen.Task
                  .deployed_as("task_under_test")
        )

        syskit_start_execution_agents(@task)
        @task.properties.node_id = 0
        @task.properties.status_use_pdo = false
        @can_in = syskit_create_writer @task.can_in_port, type: :buffer, size: 20
        @can_out = syskit_create_reader @task.can_out_port

        canopen_set_bytes 0x2113, 0x0, [0, 0] # ReadAllDigitalOutput
        canopen_set_bytes 0x210D, 0x1, [0, 0] # VoltageInternal
        canopen_set_bytes 0x210D, 0x2, [0, 0] # VoltageBattery
        canopen_set_bytes 0x210D, 0x3, [0, 0] # VoltageBattery
        canopen_set_bytes 0x2111, 0x0, [0, 0] # StatusFlagsRaw
        canopen_set_bytes 0x2112, 0x0, [0, 0] # FaultFlagsRaw
        canopen_set_bytes 0x210f, 0x1, [0, 0] # TemperatureMCU
    end

    after do
        if @task.starting? || @task.running?
            expect_canopen_interaction(@can_in, @can_out, 0) { task.stop! }
                .to_emit task.stop_event
        end
    end

    describe "GPIO handling" do
        it "reads the current state and outputs it on start" do
            @task.properties.digital_output_config = {
                gpio_paths: %w[1 3], timeout: { microseconds: 0 }, defaults: [0, 1]
            }
            canopen_set_bytes(0x2113, 0, [0, 1 << 2])

            output =
                expect_canopen_interaction(@can_in, @can_out, 0)
                .scheduler(true)
                .to_have_one_new_sample @task.digital_output_port

            assert_equal 0, output.states[0].data
            assert_equal 1, output.states[1].data
        end

        it "maintains in the output the order fo the gpio paths" do
            @task.properties.digital_output_config = {
                gpio_paths: %w[3 1], timeout: { microseconds: 0 }, defaults: [0, 1]
            }
            canopen_set_bytes(0x2113, 0, [0, 1 << 2])

            output =
                expect_canopen_interaction(@can_in, @can_out, 0)
                .scheduler(true)
                .to_have_one_new_sample @task.digital_output_port

            assert_equal 1, output.states[0].data
            assert_equal 0, output.states[1].data
        end

        it "unconditionally sets a ON default value on start" do
            @task.properties.digital_output_config = {
                gpio_paths: %w[3], timeout: { microseconds: 0 }, defaults: [1]
            }

            expect_canopen_interaction(@can_in, @can_out, 0)
                .scheduler(true)
                .to_emit task.start_event

            assert_equal [3], canopen_get_bytes(0x2009, 0)
        end

        it "unconditionally sets a OFF default value on start" do
            @task.properties.digital_output_config = {
                gpio_paths: %w[3], timeout: { microseconds: 0 }, defaults: [0]
            }

            expect_canopen_interaction(@can_in, @can_out, 0)
                .scheduler(true)
                .to_emit task.start_event

            assert_equal [3], canopen_get_bytes(0x200A, 0)
        end
    end
end