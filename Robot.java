package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commands.FollowPath;
import org.firstinspires.ftc.teamcode.commands.ShootArtifact;
import org.firstinspires.ftc.teamcode.commands.AutoShoot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.util.Alliance;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

public class Robot {

    public final Intake intake;
    public final Outtake outtake;
    public Follower follower;

    public Gamepad gamepad1, gamepad2;
    public Alliance alliance;
    public Telemetry telemetry;
    public boolean dataCollectionMode = false;

    private final List<String> shotLog = new ArrayList<>();
    private boolean align = false;
    private boolean robotCentric = true;
    private final Pose targetPosition;

    // AutoShoot telemetry values
    private double lastDistance = 0.0;
    private int lastTargetRPM = 0;

    public Robot(HardwareMap hwMap, Alliance alliance) {
        intake = new Intake(hwMap);
        outtake = new Outtake(hwMap);
        follower = Constants.createFollower(hwMap);

        this.alliance = alliance;
        LimeLight.init(hwMap);

        follower.update();
        follower.startTeleopDrive(true);

        targetPosition = alliance == Alliance.BLUE ? new Pose(0, 144) : new Pose(0, 144).mirror();
    }

    public void setGamepads(Gamepad gp1, Gamepad gp2) {
        this.gamepad1 = gp1;
        this.gamepad2 = gp2;
        configureKeyBinds();
    }

    private void configureKeyBinds() {
        if (gamepad1 != null) {
            // Shooter forward (R1 on PS4)
            dev.nextftc.bindings.Bindings.button(() -> gamepad1.right_bumper).whenTrue(() -> outtake.start.schedule());
            dev.nextftc.bindings.Bindings.button(() -> !gamepad1.right_bumper).whenTrue(() -> outtake.stop.schedule());

            // Intake forward (L1 on PS4)
            dev.nextftc.bindings.Bindings.button(() -> gamepad1.left_bumper).whenTrue(() -> intake.startCommand.schedule());
            dev.nextftc.bindings.Bindings.button(() -> !gamepad1.left_bumper).whenTrue(() -> intake.stopCommand.schedule());

            // Intake reverse (Circle on PS4)
            dev.nextftc.bindings.Bindings.button(() -> gamepad1.b).whenTrue(() -> intake.reverseCommand.schedule());
            dev.nextftc.bindings.Bindings.button(() -> !gamepad1.b).whenTrue(() -> intake.stopCommand.schedule());

            // Auto-align toggle (Square on PS4)
            dev.nextftc.bindings.Bindings.button(() -> gamepad1.x).whenTrue(() -> {
                align = !align;
                if (telemetry != null) {
                    telemetry.addLine("Auto-Align: " + (align ? "ON" : "OFF"));
                    telemetry.update();
                }
            });

            // Auto-shoot (Triangle on PS4)
            dev.nextftc.bindings.Bindings.button(() -> gamepad1.y).whenTrue(() -> {
                AutoShoot autoShoot = new AutoShoot(this);
                autoShoot.setTelemetryHook((distance, rpm) -> {
                    lastDistance = distance;
                    lastTargetRPM = rpm;
                });
                autoShoot.schedule();
            });
        }

        if (gamepad2 != null) {
            // Data collection controls
            dev.nextftc.bindings.Bindings.button(() -> gamepad2.start).whenTrue(() -> {
                dataCollectionMode = !dataCollectionMode;
                if (telemetry != null) {
                    telemetry.addLine("Data Collection Mode: " + (dataCollectionMode ? "ON" : "OFF"));
                    telemetry.update();
                }
            });
            dev.nextftc.bindings.Bindings.button(() -> gamepad2.y).whenTrue(() -> shotLog.add("HIT"));
            dev.nextftc.bindings.Bindings.button(() -> gamepad2.a).whenTrue(() -> shotLog.add("MISS"));
            dev.nextftc.bindings.Bindings.button(() -> gamepad2.back).whenTrue(() -> shotLog.clear());
            dev.nextftc.bindings.Bindings.button(() -> gamepad2.guide).whenTrue(() -> {
                if (telemetry != null) telemetry.log().add(getCalibrationDataString());
            });
        }
    }

    public void periodic() {
        follower.update();
        handleDrive(align);
        BindingManager.update();
        CommandManager.INSTANCE.run();

        // Telemetry overlay for AutoShoot
        if (telemetry != null) {
            telemetry.addData("[AutoShoot] Distance (in)", String.format("%.1f", lastDistance));
            telemetry.addData("[AutoShoot] Target RPM", lastTargetRPM);
            telemetry.update();
        }
    }

    public Command followPath(PathChain path, double maxPower) {
        return new FollowPath(path, this.follower, maxPower);
    }

    public Command shootArtifact(int shots) {
        return new ShootArtifact(this, shots);
    }

    private void handleDrive(boolean align) {
        if (gamepad1 == null) return;

        if (!align) {
            follower.setTeleOpDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, robotCentric);
            return;
        }

        LLResult result = LimeLight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx();
            double headingError = Math.toRadians(tx);
            double kP = 0.8;
            double rotationPower = kP * headingError;

            follower.setTeleOpDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, rotationPower, robotCentric);
        } else {
            follower.setTeleOpDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, robotCentric);
        }
    }

    public String getCalibrationDataString() {
        return String.format("Pose: %s | Intake: %s | Outtake: %s | Shots: %s",
                follower.getPose(), intake.getStatus(), outtake.getStatus(), shotLog.toString());
    }
}
