//check DECODE by Paipathan for more info
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.core.commands.utility.InstantCommand;

public class Intake {
    private final DcMotorEx intakeMotor;
    private final CRServo leftIntakeServo;
    private final CRServo rightIntakeServo;

    public InstantCommand startCommand;
    public InstantCommand stopCommand;
    public InstantCommand reverseCommand;

    public static boolean isBusy = false;

    // Directional gain compensation
    private static final double FWD_GAIN_MOTOR = 1.08; // slight boost forward
    private static final double REV_GAIN_MOTOR = 1.00;

    private static final double FWD_GAIN_LEFT  = 1.00;
    private static final double REV_GAIN_LEFT  = 1.00;

    private static final double FWD_GAIN_RIGHT = 1.00;
    private static final double REV_GAIN_RIGHT = 1.00;

    public Intake(HardwareMap hwMap) {
        intakeMotor = hwMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); // remove PID cap
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT); // no brake drag
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftIntakeServo = hwMap.get(CRServo.class, "leftIntakeServo");
        leftIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE); // flipped to match geometry

        rightIntakeServo = hwMap.get(CRServo.class, "rightIntakeServo");
        rightIntakeServo.setDirection(DcMotorSimple.Direction.FORWARD);

        startCommand   = new InstantCommand(this::start);
        stopCommand    = new InstantCommand(this::stop);
        reverseCommand = new InstantCommand(this::reverse);
    }

    /** Forward intake with ramp + gain */
    public void start() {
        rampPower(1.0 * FWD_GAIN_MOTOR);
        leftIntakeServo.setPower(1.0 * FWD_GAIN_LEFT);
        rightIntakeServo.setPower(1.0 * FWD_GAIN_RIGHT);
        isBusy = true;
    }

    /** Stop intake */
    public void stop() {
        intakeMotor.setPower(0.0);
        leftIntakeServo.setPower(0.0);
        rightIntakeServo.setPower(0.0);
        isBusy = false;
    }

    /** Reverse intake */
    public void reverse() {
        intakeMotor.setPower(-1.0 * REV_GAIN_MOTOR);
        leftIntakeServo.setPower(-1.0 * REV_GAIN_LEFT);
        rightIntakeServo.setPower(-1.0 * REV_GAIN_RIGHT);
        isBusy = true;
    }

    /** Gentle ramp to avoid brownout */
    private void rampPower(double target) {
        double current = intakeMotor.getPower();
        double step = 0.08; // adjust ramp speed
        if (target > current) current = Math.min(target, current + step);
        else current = Math.max(target, current - step);
        intakeMotor.setPower(current);
    }

    /** Status string for telemetry */
    public String getStatus() {
        if (isBusy) {
            return String.format("Intake Running | Motor: %.2f | Left: %.2f | Right: %.2f",
                    intakeMotor.getPower(),
                    leftIntakeServo.getPower(),
                    rightIntakeServo.getPower());
        } else {
            return "Intake Stopped";
        }
    }
}
