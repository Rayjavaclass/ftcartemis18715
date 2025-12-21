package org.firstinspires.ftc.teamcode.tutorials;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class Flywheeltuner extends OpMode {
    public DcMotorEx flywheelMotor;

    public double highVelocity = 5000;
    public double lowVelocity = 900;

    double curTargetVelocity = highVelocity;

    double F = 0;
    double P = 0;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};
    int stepIndex= 1;
    @Override
    public void init() {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheelMotor");
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setDirection(DcMotor.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("Initialization complete");
    }

    @Override
    public void loop() {
        // use this to get gamepad command, set target velocity, update telemetry

        if(gamepad1.aWasPressed()) {
            if (curTargetVelocity == highVelocity) {
                curTargetVelocity = lowVelocity;
            } else { curTargetVelocity = highVelocity; }

        }

        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
            telemetry.addData("Step Size", stepSizes[stepIndex]);
        }

        if (gamepad1.dpadLeftWasPressed()) {
            F += stepSizes[stepIndex];
            telemetry.addData("F", F);
        }

        if (gamepad1.dpadRightWasPressed()) {
            F -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        flywheelMotor.setVelocity(curTargetVelocity);

        double currentVelocity = flywheelMotor.getVelocity();
        telemetry.addData("Current Velocity", currentVelocity);
        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.update();

        double error = curTargetVelocity - currentVelocity;

        telemetry.addData("Tuning P", "% 4f (Dpad Up or Down", P);
        telemetry.addData("Tuning F", "% 4f (Dpad Left or Right", F);
        telemetry.addData("Tuning Step Size", "%.4f (B Button" ,stepSizes[stepIndex]);
        telemetry.update();


    }
}
