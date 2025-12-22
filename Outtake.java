package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.hardware.powerable.Powerable;
import dev.nextftc.hardware.powerable.SetPower;

public class Outtake {
    public static DcMotorEx BottomouttakeMotor;
    public final DcMotorEx flyWheelMotor;

    public InstantCommand start;
    public InstantCommand stop;
    public InstantCommand reverse;

    public InstantCommand autoStart;

    public static double targetRPM = 5000.0;
    public static double tolerenceRPM =100.0; //if it is within 100 rpm then be ready

    public static boolean isBusy = false;

    public static InterpLUT lut = new InterpLUT();
// this is for when hood will be added

    public Outtake (HardwareMap hwMap) {
        BottomouttakeMotor = hwMap.get(DcMotorEx.class, "outtakeMotor");
        BottomouttakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BottomouttakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BottomouttakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        flyWheelMotor = hwMap.get(DcMotorEx.class, "flyWheelMotor");
        flyWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flyWheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flyWheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        start = new InstantCommand(()-> {
            BottomouttakeMotor.setPower(1);
            flyWheelMotor.setPower(-1);
            isBusy = true;

        });

        reverse = new InstantCommand(() -> {
            BottomouttakeMotor.setPower(-1);
            flyWheelMotor.setPower(1);
            isBusy = true;
        });

        stop = new InstantCommand(()-> {
            BottomouttakeMotor.setPower(0);
            flyWheelMotor.setPower(0);
            isBusy = false;
        });
    }

    public static double getTopRPM() {
        return BottomouttakeMotor.getVelocity(AngleUnit.DEGREES) / 6.0;
    }

    public double getFlywheelRPM() {
        return flyWheelMotor.getVelocity(AngleUnit.DEGREES) / 6.0;
    }

    public boolean isReadyToShoot() {
        double currentRPM = getTopRPM();
        return Math.abs(currentRPM - targetRPM) <+ tolerenceRPM;

    }

    public static double getRPMPercentage() {
        return (getTopRPM() / targetRPM) * 100;
    }

    public void updateTelemetry(Telemetry telemetry) {
        double currentRPM = getTopRPM();
        double flyWheelRPM = getFlywheelRPM();
        boolean ready = isReadyToShoot();
        telemetry.addData("Outtake RPM", currentRPM);
        telemetry.addData("Flywheel RPM", flyWheelRPM);
        telemetry.addData("Flywheel RPM %", getFlywheelRPM());
        telemetry.addData("Ready to Shoot", ready);
        telemetry.addData("Outtake RPM %", getRPMPercentage());
    }


}
