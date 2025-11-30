package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Alliance;

@TeleOp(name="BlueTeleOp12", group="Competition")
public class BlueTeleOp extends OpMode {

    private Robot robot;

    @Override
    public void init() {
        // Initialize robot with BLUE alliance
        robot = new Robot(hardwareMap, Alliance.BLUE);
        robot.telemetry = telemetry;
    }

    @Override
    public void start() {
        // Bind gamepads
        robot.setGamepads(gamepad1, gamepad2);
    }

    @Override
    public void loop() {
        // Run periodic updates
        robot.periodic();

        // Telemetry overlays
        telemetry.addLine("=== DRIVER OVERLAY ===");
        telemetry.addData("Alliance", robot.alliance);
        telemetry.addData("Pose", robot.follower.getPose());
        telemetry.addData("Shooter Status", robot.outtake.getStatus());
        telemetry.addData("Intake Status", robot.intake.getStatus());
        telemetry.addData("Auto-Align", robot.alliance + " | " + (robot.telemetry != null ? "ON/OFF" : "N/A"));
        telemetry.addData("Data Collection Mode", robot.dataCollectionMode ? "ON" : "OFF");
        telemetry.update();
    }
}
