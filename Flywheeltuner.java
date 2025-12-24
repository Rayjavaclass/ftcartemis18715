package org.firstinspires.ftc.teamcode.tutorials;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class Flywheeltuner extends OpMode {
    public DcMotorEx flywheelMotor;

    public double highRPM = 6000;
    public double lowRPM = 900;

    double curTargetRPM = highRPM;

    double F = 2;
    double P = 2.5;

    double[] stepSizes = {6000, 5000, 4000, 3000};
    int stepIndex = 1;

    boolean lastA, lastB, lastDpadLeft, lastDpadRight, lastDpadUp, lastDpadDown;

    final double TICKS_PER_REV = 28;
    double m_counter = 0;

    @Override
    public void init() {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheelMotor");
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setDirection(DcMotor.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {

        boolean aPressed = gamepad1.a && !lastA;
        boolean bPressed = gamepad1.b && !lastB;
        boolean dpadLeftPressed = gamepad1.dpad_left && !lastDpadLeft;
        boolean dpadRightPressed = gamepad1.dpad_right && !lastDpadRight;
        boolean dpadUpPressed = gamepad1.dpad_up && !lastDpadUp;
        boolean dpadDownPressed = gamepad1.dpad_down && !lastDpadDown;

        telemetry.addData("Counter: ", m_counter);
        m_counter++;

        if (aPressed) {
            curTargetRPM = (curTargetRPM == highRPM) ? lowRPM : highRPM;
            telemetry.addData("aPressed, curTargetRPM: ", curTargetRPM);
        }

        if (bPressed) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
            telemetry.addData("bPressed, curTargetRPM: ", curTargetRPM);
        }

        if (dpadLeftPressed)
        {
            curTargetRPM -= stepSizes[stepIndex];
            telemetry.addData("D-Pad Left Pressed, curTargetRPM: ", curTargetRPM);
        }

        if (dpadRightPressed)
        {
            curTargetRPM += stepSizes[stepIndex];
            telemetry.addData("D-Pad Right Pressed, curTargetRPM: ", "%.0f", curTargetRPM);
        }

        if (curTargetRPM < 0) curTargetRPM = 0;

        if (dpadUpPressed) P += 0.1;
        if (dpadDownPressed) P -= 0.1;

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        double targetTicksPerSec = (curTargetRPM * TICKS_PER_REV) / 60.0;
        telemetry.addData("Just before setting the velocity targetTicksPerSec: ", "%.0f", targetTicksPerSec, " curTargetRPM: ", "%.0f", curTargetRPM);
        flywheelMotor.setVelocity(targetTicksPerSec);

        double curVelocity = flywheelMotor.getVelocity();
        telemetry.addData("Current Velocity (Ticks/Sec)", "%.2f", curVelocity);

        double currentRPM = (curVelocity * 60.0) / TICKS_PER_REV;

        telemetry.addData("Target RPM", "%.0f", curTargetRPM);
        telemetry.addData("Current RPM", "%.0f", currentRPM);
        telemetry.addData("Target RPM", "%.0f", curTargetRPM);
        telemetry.addData("Error RPM", "%.0f", curTargetRPM - currentRPM);
        telemetry.addLine();
        telemetry.addData("P", "%.3f", P);
        telemetry.addData("F", "%.3f", F);
        telemetry.addData("Step Size (RPM)", "%.0f", stepSizes[stepIndex]);

        telemetry.update();

        lastA = gamepad1.a;
        lastB = gamepad1.b;
        lastDpadLeft = gamepad1.dpad_left;
        lastDpadRight = gamepad1.dpad_right;
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
    }
}
