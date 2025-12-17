package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple; // Import needed for reverse
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;

public class Outtake implements Subsystem {

    // Hardware definitions
    public DcMotorEx topMotor;            // Changed name to match Robot.java
    public DcMotorEx bottomOuttakeMotor;  // Changed name to match Robot.java

    public Servo turretServo;
    public Servo feedServo;
    public Servo hood; // Added hood based on your Robot.java usage

    // Turret Tuning
    private final double TURRET_CENTER_POS = 0.5;
    private final double TURRET_MAX_DEGREES = 300.0;

    // Hood constants
    public static final double MIN = 0.0;
    public static final double MAX = 1.0;

    // Commands defined as fields (Standard NextFTC practice)
    public final Command start;
    public final Command stop;
    public final Command reverse;

    public Outtake(HardwareMap hardwareMap) {
        // Initialize Hardware
        turretServo = hardwareMap.get(Servo.class, "turret");
        feedServo = hardwareMap.get(Servo.class, "feeder");
        hood = hardwareMap.get(Servo.class, "hood"); // Make sure config has "hood"

        topMotor = hardwareMap.get(DcMotorEx.class, "topOuttake"); // Verify config name!
        bottomOuttakeMotor = hardwareMap.get(DcMotorEx.class, "bottomOuttake"); // Verify config name!

        // Setup motors
        // Usually one motor needs to be reversed for shooters to spin correctly
        // topMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Define Commands
        start = new InstantCommand(this::shooterStart);
        stop = new InstantCommand(this::shooterStop);
        reverse = new InstantCommand(this::shooterReverse);
    }

    @Override
    public void periodic() {
        // Required by NextFTC Subsystem
    }

    // =========================================================
    //  METHODS FOR MOTORS
    // =========================================================

    public void shooterStart() {
        // "Both outtake needs to be running"
        topMotor.setPower(0.6);
        bottomOuttakeMotor.setPower(0.6);
    }

    public void shooterStop() {
        topMotor.setPower(0.0);
        bottomOuttakeMotor.setPower(0.0);
    }

    public void shooterReverse() {
        topMotor.setPower(-0.3);
        bottomOuttakeMotor.setPower(-0.3);
    }

    // =========================================================
    //  METHODS FOR TURRET / SENSORS
    // =========================================================

    public double getTopRPM() {
        // Convert ticks/sec to RPM (approximate, usually ticks * 60 / encoder_resolution)
        // Just returning velocity for logic checks
        return topMotor.getVelocity();
    }

    public double getTurretAngle() {
        double currentPos = turretServo.getPosition();
        return (currentPos - TURRET_CENTER_POS) * TURRET_MAX_DEGREES;
    }

    public void setTurretAngle(double degrees) {
        double clampedDegrees = Math.max(-90, Math.min(90, degrees));
        double targetPos = TURRET_CENTER_POS + (clampedDegrees / TURRET_MAX_DEGREES);
        turretServo.setPosition(targetPos);
    }

    public boolean isFlywheelReady() {
        // Check if both motors are running fast enough
        return topMotor.getPower() > 0.5 && bottomOuttakeMotor.getPower() > 0.5;
    }

    public void feedBall() {
        feedServo.setPosition(1.0); // Push
        // Note: You need logic to retract this (0.0) after a split second
    }
}
