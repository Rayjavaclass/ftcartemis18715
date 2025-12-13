package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.hardware.limelightvision.LLResult;
import dev.nextftc.core.commands.Command;import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.LimeLight;
import org.firstinspires.ftc.teamcode.Robot;

public class AutoShoot extends Command {

    private final Robot robot;
    private final ElapsedTime timer = new ElapsedTime();

    // --- TUNING CONSTANTS ---
    private static final double TURRET_TOLERANCE_DEGREES = 2.0;
    private static final double FEED_EXTENSION_TIME = 0.25;
    private static final double FEED_RETRACT_TIME = 0.25;

    // --- STATE MACHINE ---
    private enum ShootState {
        AIMING,
        FEEDING,
        RESETTING
    }

    private ShootState currentState = ShootState.AIMING;

    public AutoShoot(Robot robot) {
        this.robot = robot;
        addRequirements(robot.outtake);
    }

    @Override
    public void start() {
        // Start the flywheels immediately when command starts
        robot.outtake.shooterStart();
        currentState = ShootState.AIMING;
    }

    @Override
    public void update() {
        // 1. Get Vision Data
        LLResult result = LimeLight.getLatestResult();

        // 2. Aim Turret
        boolean isAimed = false;

        if (result != null && result.isValid()) {
            double tx = result.getTx();

            double currentAngle = robot.outtake.getTurretAngle();
            double targetAngle = currentAngle - tx;

            robot.outtake.setTurretAngle(targetAngle);

            isAimed = Math.abs(tx) < TURRET_TOLERANCE_DEGREES;
        }

        // 3. Handle State Machine
        switch (currentState) {
            case AIMING:
                if (isAimed && robot.outtake.isFlywheelReady()) {
                    robot.outtake.feedRing();
                    timer.reset();
                    currentState = ShootState.FEEDING;
                }
                break;

            case FEEDING:
                if (timer.seconds() > FEED_EXTENSION_TIME) {
                    robot.outtake.feedServo.setPosition(0.0);
                    timer.reset();
                    currentState = ShootState.RESETTING;
                }
                break;

            case RESETTING:
                if (timer.seconds() > FEED_RETRACT_TIME) {
                    currentState = ShootState.AIMING;
                }
                break;
        }
    }

    @Override
    public void stop(boolean interrupted) {
        robot.outtake.shooterStop();
        robot.outtake.feedServo.setPosition(0.0);
    }

    @Override
    public boolean isDone() {
        // Returns false so the command runs continuously while the button is held
        return false;
    }
}
