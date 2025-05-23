# frozen_string_literal: true

using_task_library "motors_roboteq_canopen"

describe OroGen.motors_roboteq_canopen.DigitalOutMultiplexerTask do
    run_live

    attr_reader :task

    before do
        @task = syskit_deploy(
            OroGen.motors_roboteq_canopen.DigitalOutMultiplexerTask
                  .deployed_as("task_under_test")
        )
    end

    it "writes the clutch and controller selection commands in the right index" do
        t = syskit_configure_and_start(@task)

        clutch_cmd =
            Types.linux_gpios.GPIOState.new(states: [Types.raw_io.Digital.new(data: 1)])
        controller_selection_cmd =
            Types.linux_gpios.GPIOState.new(states: [Types.raw_io.Digital.new(data: 0)])
        time_before = Time.now
        expect_execution do
            syskit_write t.clutch_command_port, clutch_cmd
            syskit_write t.controller_selection_command_port, controller_selection_cmd
        end.to do
            have_one_new_sample(t.digital_out_command_port).matching do |cmd|
                assert_operator cmd.time, :>, time_before
                assert_operator Time.now, :>, cmd.time
                assert_equal cmd.states[0].data, 1
                assert_equal cmd.states[1].data, 0
            end
        end
    end

    it "raises an exception when the clutch command doesnt have the expected size" do
        t = syskit_configure_and_start(@task)

        clutch_cmd =
            Types.linux_gpios.GPIOState.new(states: [])
        controller_selection_cmd =
            Types.linux_gpios.GPIOState.new(states: [Types.raw_io.Digital.new(data: 0)])
        expect_execution do
            syskit_write t.clutch_command_port, clutch_cmd
            syskit_write t.controller_selection_command_port, controller_selection_cmd
        end.to do
            emit t.exception_event
        end
    end

    it "raises an exception when the controller_selection command doesnt have the " \
       "expected size" do
        t = syskit_configure_and_start(@task)

        clutch_cmd =
            Types.linux_gpios.GPIOState.new(states: [Types.raw_io.Digital.new(data: 1)])
        controller_selection_cmd =
            Types.linux_gpios.GPIOState.new(states: [])
        expect_execution do
            syskit_write t.clutch_command_port, clutch_cmd
            syskit_write t.controller_selection_command_port, controller_selection_cmd
        end.to do
            emit t.exception_event
        end
    end
end
