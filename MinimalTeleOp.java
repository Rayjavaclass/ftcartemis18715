package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Alliance;

@TeleOp(name="MinimalTeleOp", group="Test")
public class MinimalTeleOp extends OpMode {

    private Robot robot;

    @Override
    public void init() {
        // Initialize robot with BLUE alliance (swap to RED if needed)
        robot = new Robot(hardwareMap, Alliance.BLUE);
        robot.telemetry = telemetry;
    }

    @Override
    public void start() {
        // Bind gamepads so button mappings activate
        robot.setGamepads(gamepad1, gamepad2);
    }

    @Override
    public void loop() {
        // Run periodic updates (drive, intake, auto-align, auto-shoot)
        robot.periodic();

        // Debug telemetry overlay
        telemetry.addLine("=== Minimal TeleOp Debug ===");
        telemetry.addData("Intake Power", robot.intake.getStatus());
        telemetry.addData("Shooter RPM", robot.outtake.getTopRPM());
        telemetry.addData("AutoAlign", robot.alliance + " | " + (robot.telemetry != null ? "ON/OFF" : "N/A"));
        telemetry.update();
    }
}
