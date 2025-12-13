package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.LimeLight;
import org.firstinspires.ftc.teamcode.Robot;

// Pedro Pathing Import
import com.pedropathing.geometry.Pose;

import dev.nextftc.core.commands.Command;

public class AutoAlign extends Command {
    private final Robot robot;

    // --- Tuning Constants ---
    private static final double TURN_KP_VISION = 0.03;
    private static final double DRIVE_KP_VISION = 0.04;
    private static final double TURN_KP_ODO = 1.5;

    private static final double MAX_AUTO_SPEED = 0.5;
    private static final double DESIRED_TARGET_Y = 0.0;

    // --- Tag Locations ---
    // Update these coordinates based on your specific field setup!
    private static final Pose BLUE_TAG_POSE = new Pose(60, 48, 0);
    private static final Pose RED_TAG_POSE = new Pose(60, -48, 0);

    public AutoAlign(Robot robot) {
        this.robot = robot;
        setInterruptible(true);

        // NOTE: We cannot addRequirements(robot.follower) because Pedro's Follower
        // does not implement the NextFTC Subsystem interface.
        // Ensure no other driver controls are active while this command runs.
    }

    @Override
    public void update() {
        LLResult result = LimeLight.getLatestResult();

        // 1. Check if we see the CORRECT tag for our alliance
        boolean targetVisible = false;
        if (result != null && result.isValid() && !result.getFiducialResults().isEmpty()) {
            int detectedId = result.getFiducialResults().get(0).getFiducialId();
            // Blue=2, Red=5 (Example IDs - check your season manual!)
            int targetId = (robot.alliance == Alliance.BLUE) ? 2 : 5;

            if (detectedId == targetId) {
                targetVisible = true;
            }
        }

        if (targetVisible) {
            runVisionControl(result);
        } else {
            runOdometryControl();
        }
    }

    private void runVisionControl(LLResult result) {
        // Vision Logic (Robot Centric)
        double tx = result.getTx();
        double headingError = -tx; // Invert if turning wrong way
        double turnPower = headingError * TURN_KP_VISION;

        double ty = result.getTy();
        double distanceError = ty - DESIRED_TARGET_Y;
        double drivePower = distanceError * DRIVE_KP_VISION;

        // Clip Speed
        turnPower = Range.clip(turnPower, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        drivePower = Range.clip(drivePower, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);

        // FIX: Use setTeleOpDrive with TRUE for Robot Centric
        robot.follower.setTeleOpDrive(
                drivePower,                  // Forward/Back
                -robot.gamepad.left_stick_x, // Manual Strafe
                turnPower,                   // Auto Turn
                true                         // TRUE = Robot Centric
        );
    }

    private void runOdometryControl() {
        // Odometry Logic (Field Centric)
        Pose targetPose = (robot.alliance == Alliance.BLUE) ? BLUE_TAG_POSE : RED_TAG_POSE;
        Pose currentPose = robot.follower.getPose();

        // Calculate angle to target
        double diffX = targetPose.getX() - currentPose.getX();
        double diffY = targetPose.getY() - currentPose.getY();
        double targetAngle = Math.atan2(diffY, diffX);

        // Calculate turn power
        double currentHeading = currentPose.getHeading();
        double angleDiff = normalizeAngle(targetAngle - currentHeading);

        double turnPower = angleDiff * TURN_KP_ODO;
        turnPower = Range.clip(turnPower, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);

        // FIX: Use setTeleOpDrive with FALSE for Field Centric
        robot.follower.setTeleOpDrive(
                -robot.gamepad.left_stick_y, // Manual Forward
                -robot.gamepad.left_stick_x, // Manual Strafe
                turnPower,                   // Auto Turn
                false                        // FALSE = Field Centric
        );
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    @Override
    public boolean isDone() {
        // Command runs until interrupted (button release)
        return false;
    }
}
