package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.hardware.limelightvision.LLResult;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.LimeLight;
import org.firstinspires.ftc.teamcode.Robot;
import dev.nextftc.core.commands.Command;
import com.pedropathing.geometry.Pose;

public class AutoShoot extends Command {

    private final Robot robot;


    private static final double TURRET_KP = 0.015; // Proportional gain for turret centering
    private static final double MAX_TURRET_RANGE = 90.0; // Degrees the turret can turn left/right

    // State variables
    private int shotsFired = 0;
    private long lastShotTime = 0;

    public AutoShoot(Robot robot) {
        this.robot = robot;
        setInterruptible(true);
        addRequirements(robot.outtake);
    }

    @Override
    public void start() {
        shotsFired = 0;
        robot.outtake.shooterStart(); // Spin up flywheel
    }

    @Override
    public void update() {
        LLResult result = LimeLight.getLatestResult();

        // 1. Target Validation
        boolean validTarget = false;
        if (result != null && result.isValid()) {
            // need to add ID check here if specific tags matter (like Blue vs Red)
            validTarget = true;
        }

        // 2. Aiming Logic (Turret)
        if (validTarget) {
            double tx = result.getTx();

            // Calculate new turret angle relative to current
            // If tx is +10 deg (target is right), we need to add 10 deg to current turret angle
            double currentTurretAngle = robot.outtake.getTurretAngle();
            double targetTurretAngle = currentTurretAngle - tx;

            robot.outtake.setTurretAngle(targetTurretAngle);

        } else {
           //If no target, center the turret
            robot.outtake.setTurretAngle(0);
        }


        boolean isAimed = validTarget && Math.abs(result.getTx()) < 2.0;
        boolean isReady = robot.outtake.isFlywheelReady();

        if (isAimed && isReady && (System.currentTimeMillis() - lastShotTime > 500)) {
            robot.outtake.feedRing(); // Push ring into flywheel
            lastShotTime = System.currentTimeMillis();
            shotsFired++;
        }
    }

    @Override
    public boolean isDone() {
        // stops after 3 shots or if button released
        return shotsFired >= 3;
    }

    public void end(boolean interrupted) {
        robot.outtake.shooterStop();
        robot.outtake.setTurretAngle(0); // Recenter turret when completed
    }
}
