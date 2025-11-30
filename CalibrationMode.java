package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Drawing;

import dev.nextftc.core.commands.CommandManager;

/**
 * Calibration Mode - Systematic Testing OpMode
 *
 * This OpMode helps you systematically test shooting from exact distances.
 * It guides you through testing each distance with multiple shots.
 *
 * WORKFLOW:
 * 1. Drive robot to target distance shown on screen
 * 2. Shoot 5-10 times from that distance
 * 3. Mark each shot as hit (△) or miss (✕)
 * 4. Use D-Pad Left/Right to move to next test distance
 * 5. Repeat for all distances
 * 6. Press SHARE to export all data
 *
 * CONTROLS (PS4):
 *   D-Pad Left/Right = Previous/Next test distance
 *   L2 = Auto-align and shoot
 *   L1 = Intake
 *   R1 = Shooter
 *   △ Triangle = Mark HIT
 *   ✕ Cross = Mark MISS
 *   ◻ Square = Clear all data
 *   SHARE (GP2) = Export data
 */
@TeleOp(name="6Calibration Mode", group="Testing")
public class CalibrationMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, Alliance.BLUE);
        robot.telemetry = telemetry;
        robot.dataCollectionMode = true; // Always in data collection mode

        CommandManager.INSTANCE.cancelAll();

        // Test distances (in meters)
        double[] testDistances = {0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5};
        int currentTestIndex = 0;

        // ═══════════════════════════════════════════════════════════════
        // INITIALIZATION
        // ═══════════════════════════════════════════════════════════════
        telemetry.addLine("═══════════════════════════════════════");
        telemetry.addLine("🔬 CALIBRATION MODE");
        telemetry.addLine("═══════════════════════════════════════");
        telemetry.addLine("");
        telemetry.addLine("This mode helps you systematically");
        telemetry.addLine("test shooting from exact distances.");
        telemetry.addLine("");
        telemetry.addLine("WORKFLOW:");
        telemetry.addLine("1. Drive to target distance on screen");
        telemetry.addLine("2. Hold L2 to auto-aim and shoot");
        telemetry.addLine("3. Press △ (hit) or ✕ (miss)");
        telemetry.addLine("4. Repeat 5-10 times per distance");
        telemetry.addLine("5. D-Pad Left/Right = Next distance");
        telemetry.addLine("6. Press SHARE (GP2) to export data");
        telemetry.addLine("");
        telemetry.addLine("Press START when ready");
        telemetry.addLine("═══════════════════════════════════════");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            // ═══════════════════════════════════════════════════════════════
            // MAIN ROBOT UPDATE
            // ═══════════════════════════════════════════════════════════════
            robot.periodic();

            // ═══════════════════════════════════════════════════════════════
            // DISTANCE NAVIGATION
            // ═══════════════════════════════════════════════════════════════
            if (gamepad1.dpad_left && currentTestIndex > 0) {
                currentTestIndex--;
                sleep(300); // Debounce
            }
            if (gamepad1.dpad_right && currentTestIndex < testDistances.length - 1) {
                currentTestIndex++;
                sleep(300); // Debounce
            }

            // ═══════════════════════════════════════════════════════════════
            // EXPORT DATA (PS4: SHARE button on GP2)
            // ═══════════════════════════════════════════════════════════════
            if (gamepad2.back) { // SHARE button on PS4
                String data = robot.getCalibrationDataString();
                telemetry.log().add(data);

                telemetry.addLine("");
                telemetry.addLine("✓ Data exported to logcat!");
                telemetry.addLine("Check Android Studio Logcat window");
                telemetry.update();
                sleep(2000);
            }

            // ═══════════════════════════════════════════════════════════════
            // CUSTOM TELEMETRY FOR CALIBRATION MODE
            // ═══════════════════════════════════════════════════════════════
            telemetry.addLine("");
            telemetry.addLine("═══════════════════════════════════════");
            telemetry.addData("🎯 TARGET DISTANCE",
                    String.format("%.1f meters", testDistances[currentTestIndex]));
            telemetry.addData("📊 Test Progress",
                    String.format("%d / %d distances", currentTestIndex + 1, testDistances.length));

            // Count shots at this distance
            int shotsAtThisDistance = countShotsNearDistance(
                    robot, testDistances[currentTestIndex], 0.2);
            telemetry.addData("Shots at this distance", shotsAtThisDistance);

            telemetry.addLine("───────────────────────────────────────");
            telemetry.addLine("CONTROLS:");
            telemetry.addLine("  D-Pad ← = Previous distance");
            telemetry.addLine("  D-Pad → = Next distance");
            telemetry.addLine("  L2 = Auto-aim and shoot");
            telemetry.addLine("  △ = Mark HIT");
            telemetry.addLine("  ✕ = Mark MISS");
            telemetry.addLine("  SHARE (GP2) = Export data");
            telemetry.addLine("═══════════════════════════════════════");

            // ═══════════════════════════════════════════════════════════════
            // FIELD VISUALIZATION
            // ═══════════════════════════════════════════════════════════════
            Drawing.init();
            Drawing.drawRobot(robot.follower.getPose());
            Drawing.drawPoseHistory(robot.follower.getPoseHistory());
            Drawing.sendPacket();
        }

        // ═══════════════════════════════════════════════════════════════
        // CLEANUP: Print final summary
        // ═══════════════════════════════════════════════════════════════
        String finalData = robot.getCalibrationDataString();
        telemetry.log().add("\n\n═══ CALIBRATION COMPLETE ═══\n" + finalData);

        telemetry.addLine("");
        telemetry.addLine("═══════════════════════════════════════");
        telemetry.addLine("CALIBRATION COMPLETE!");
        telemetry.addLine("Final data printed to logcat");
        telemetry.addLine("═══════════════════════════════════════");
        telemetry.update();
        sleep(3000);
    }

    /**
     * Helper method to count shots near a target distance
     * @param robot Robot instance
     * @param targetDistance Distance in meters
     * @param tolerance Tolerance in meters (e.g., 0.2 = within 20cm)
     * @return Number of shots within tolerance
     */
    private int countShotsNearDistance(Robot robot, double targetDistance, double tolerance) {
        int count = 0;
        String dataStr = robot.getCalibrationDataString();

        // Simple parsing - count shots near target distance
        // This is a rough estimate, not perfect
        String[] lines = dataStr.split("\n");
        for (String line : lines) {
            if (line.contains("m |")) {
                try {
                    String distStr = line.substring(0, line.indexOf("m |")).trim();
                    double distance = Double.parseDouble(distStr);
                    if (Math.abs(distance - targetDistance) <= tolerance) {
                        count++;
                    }
                } catch (Exception e) {
                    // Ignore parsing errors
                }
            }
        }

        return count;
    }
}
