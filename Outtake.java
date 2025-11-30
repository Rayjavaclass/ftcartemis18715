package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.core.commands.utility.InstantCommand;

public class Outtake {
    public final DcMotorEx topMotor;
    public final DcMotorEx bottomMotor;
    public InstantCommand start;
    public InstantCommand stop;
    public Servo hood;
    public static boolean isBusy = false;

    public Outtake(HardwareMap hwMap) {
        topMotor = hwMap.get(DcMotorEx.class, "topOuttakeMotor");
        topMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        topMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        topMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        bottomMotor = hwMap.get(DcMotorEx.class, "bottomOuttakeMotor");
        bottomMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        bottomMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        bottomMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        hood = hwMap.get(Servo.class, "hoodServo");
        hood.setDirection(Servo.Direction.REVERSE);

        start = new InstantCommand(this::shoot);
        stop = new InstantCommand(this::stopShooter);
    }

    public void shoot() {
        topMotor.setPower(1.0);
        bottomMotor.setPower(1.0);
        isBusy = true;
    }

    public void stopShooter() {
        topMotor.setPower(0.0);
        bottomMotor.setPower(0.0);
        isBusy = false;
    }

    public double getTopTPS() { return topMotor.getVelocity(); }
    public double getBottomTPS() { return bottomMotor.getVelocity(); }
    public int getTopRPM() { return (int)((topMotor.getVelocity()*60)/topMotor.getMotorType().getTicksPerRev()); }
    public int getBottomRPM() { return (int)((bottomMotor.getVelocity()*60)/bottomMotor.getMotorType().getTicksPerRev()); }

    public String getStatus() {
        return isBusy ? String.format("Shooter Running | TopTPS: %.1f | BottomTPS: %.1f", getTopTPS(), getBottomTPS())
                : "Shooter Stopped";
    }
}
