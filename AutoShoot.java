package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.LimeLight;
import org.firstinspires.ftc.teamcode.Robot;
import com.qualcomm.hardware.limelightvision.LLResult;
import dev.nextftc.core.commands.Command;

import java.util.function.BiConsumer;

public class AutoShoot extends Command {
    private final Robot robot;
    private boolean done = false;

    // Hook for telemetry feedback
    private BiConsumer<Double, Integer> telemetryHook;

    public AutoShoot(Robot robot) {
        this.robot = robot;
        setInterruptible(true);
    }

    @Override
    public void start() {
        done = false;
    }

    @Override
    public void update() {
        LLResult result = LimeLight.getLatestResult();
        if (result != null && result.isValid()) {
            double ty = result.getTy(); // vertical offset from Limelight
            double distance = calculateDistance(ty);

            // Map distance to RPM
            int targetRPM = getTargetRPM(distance);

            // Spin shooter motors at target RPM
            robot.outtake.topMotor.setVelocity(targetRPM);
            robot.outtake.bottomMotor.setVelocity(targetRPM);

            // Telemetry feedback
            if (telemetryHook != null) {
                telemetryHook.accept(distance, targetRPM);
            }

            // If shooter is at speed, feed ball
            if (robot.outtake.getTopRPM() >= targetRPM - 50) {
                robot.intake.startCommand.schedule();
                done = true;
            }
        }
    }

    @Override
    public boolean isDone() {
        return done;
    }

    @Override
    public void stop(boolean interrupted) {
        robot.intake.stopCommand.schedule();
        robot.outtake.stop.schedule();
    }

    /** Allow Robot.java to hook telemetry values */
    public void setTelemetryHook(BiConsumer<Double, Integer> hook) {
        this.telemetryHook = hook;
    }

    /** Distance calculation from Limelight vertical offset */
    private double calculateDistance(double ty) {
        double cameraHeight = 12;   // inches (adjust to your robot)
        double targetHeight = 36;   // inches (goalpost height)
        double cameraAngle = Math.toRadians(25); // degrees (adjust to your mount)
        return (targetHeight - cameraHeight) / Math.tan(cameraAngle + Math.toRadians(ty));
    }

    /** Map distance to RPM (tune experimentally) */
    private int getTargetRPM(double distance) {
        if (distance < 24) return 1000;
        else if (distance < 36) return 1200;
        else return 1400;
    }
}
