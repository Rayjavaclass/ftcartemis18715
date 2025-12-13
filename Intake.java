package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;

public class Intake {
    private final DcMotor intakeMotor;

    private final CRServo leftIntakeServo;
    private final CRServo rightIntakeServo;

    public InstantCommand start;
    public InstantCommand stop;
    public InstantCommand reverse;

    public static boolean isBusy = false;

    public Intake(HardwareMap hwMap) {
        intakeMotor = hwMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftIntakeServo = hwMap.get(CRServo.class, "leftIntakeServo");
        leftIntakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
        rightIntakeServo = hwMap.get(CRServo.class, "rightIntakeServo");
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);

        start = new InstantCommand(()-> {
            intakeMotor.setPower(1);
            leftIntakeServo.setPower(1);
            rightIntakeServo.setPower(1);

            isBusy = true;
        });

        stop = new InstantCommand(()-> {
            intakeMotor.setPower(0);
            leftIntakeServo.setPower(0);
            rightIntakeServo.setPower(0);

            isBusy = false;
        });

        reverse = new InstantCommand(()-> {
            intakeMotor.setPower(-1);
            leftIntakeServo.setPower(-1);
            rightIntakeServo.setPower(-1);

            isBusy = true;
        });
    }
}
