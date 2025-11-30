package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="IntakeTest", group="Test")
public class IntakeTest extends LinearOpMode {
    private DcMotorEx intakeMotor;
    private CRServo leftIntakeServo, rightIntakeServo;

    @Override
    public void runOpMode() throws InterruptedException {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        leftIntakeServo = hardwareMap.get(CRServo.class, "leftIntakeServo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "rightIntakeServo");

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.left_bumper) {
                intakeMotor.setPower(1.0);
                leftIntakeServo.setPower(1.0);
                rightIntakeServo.setPower(1.0);
            } else if (gamepad1.b) {
                intakeMotor.setPower(-1.0);
                leftIntakeServo.setPower(-1.0);
                rightIntakeServo.setPower(-1.0);
            } else {
                intakeMotor.setPower(0.0);
                leftIntakeServo.setPower(0.0);
                rightIntakeServo.setPower(0.0);
            }

            telemetry.addData("Motor Power", intakeMotor.getPower());
            telemetry.addData("Left Servo Power", leftIntakeServo.getPower());
            telemetry.addData("Right Servo Power", rightIntakeServo.getPower());
            telemetry.update();
        }
    }
}
