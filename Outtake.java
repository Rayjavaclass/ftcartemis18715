package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor; 
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;

public class Outtake implements Subsystem {

    // --- HARDWARE ---
    public DcMotorEx topMotor;
    public DcMotorEx bottomOuttakeMotor;
    
    // CHANGED: Turret is now a Motor
    public DcMotorEx turretMotor; 
    
    public Servo feedServo;
    public Servo hood;

    
    private final double TICKS_PER_DEGREE = 2.5; 
    private final double TURRET_MAX_ANGLE = 90.0; // Don't turn past this left/right

    // Shooter Tuning (5203 1:1)
    private static final double SHOOTER_TICKS_PER_REV = 28.0; 

    public static final double HOOD_MIN = 0.0;
    public static final double HOOD_MAX = 1.0;

    // --- COMMANDS ---
    public final Command start;
    public final Command stop;
    public final Command reverse;

    public Outtake(HardwareMap hardwareMap) {
        // --- INITIALIZE HARDWARE ---
        feedServo = hardwareMap.get(Servo.class, "feeder");
        hood = hardwareMap.get(Servo.class, "hood");

        topMotor = hardwareMap.get(DcMotorEx.class, "topOuttake");
        bottomOuttakeMotor = hardwareMap.get(DcMotorEx.class, "bottomOuttake");
        
        // CHANGED: Get Turret Motor
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");

        // --- SHOOTER MOTOR SETUP ---
        topMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomOuttakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bottomOuttakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Reverse one if needed
        topMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // --- TURRET MOTOR SETUP ---
        // We use RUN_TO_POSITION so the motor holds its angle like a servo
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Important to hold position
        turretMotor.setPower(1.0); // Allow full speed to get to position

        // --- COMMANDS ---
        start = new InstantCommand(this::shooterStart);
        stop = new InstantCommand(this::shooterStop);
        reverse = new InstantCommand(this::shooterReverse);
    }

    @Override
    public void periodic() {
        // Required by NextFTC Subsystem
    }

    // =========================================================
    //  SHOOTER METHODS
    // =========================================================

    public void shooterStart() {
        // Set power high for shooting
        topMotor.setPower(0.65);
        bottomOuttakeMotor.setPower(0.65);
    }

    public void shooterStop() {
        topMotor.setPower(0.0);
        bottomOuttakeMotor.setPower(0.0);
    }

    public void shooterReverse() {
        topMotor.setPower(-0.3);
        bottomOuttakeMotor.setPower(-0.3);
    }

    public double getTopRPM() {
        double ticksPerSecond = topMotor.getVelocity();
        return (ticksPerSecond * 60.0) / SHOOTER_TICKS_PER_REV;
    }

    public boolean isFlywheelReady() {
        // Check if actually spinning fast
        return getTopRPM() > 2500;
    }

    // =========================================================
    //  TURRET METHODS (MOTOR VERSION)
    // =========================================================

    public double getTurretAngle() {
        // Convert current encoder ticks -> degrees
        return turretMotor.getCurrentPosition() / TICKS_PER_DEGREE;
    }

    public void setTurretAngle(double degrees) {
        // 1. Clamp degrees so we don't break wires
        double clamped = Math.max(-TURRET_MAX_ANGLE, Math.min(TURRET_MAX_ANGLE, degrees));
        
        // 2. Convert degrees -> encoder ticks
        int targetTicks = (int) (clamped * TICKS_PER_DEGREE);

        // 3. Set target
        turretMotor.setTargetPosition(targetTicks);
        // Ensure mode is correct (in case it got reset)
        if (turretMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turretMotor.setPower(1.0);
        }
    }

    

    public void feedRing() {
        feedServo.setPosition(1.0); // Push
    }
}
