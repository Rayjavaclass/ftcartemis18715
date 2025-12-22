package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.hardware.powerable.Powerable;
import dev.nextftc.hardware.powerable.SetPower;

public class Outtake {
    public static DcMotorEx BottomouttakeMotor;
    public final DcMotorEx flyWheelMotor;
    Limelight3A limelight;


    public InstantCommand start;
    public InstantCommand stop;
    public InstantCommand reverse;

    public InstantCommand autoStart;

    public static double targetRPM = 5000.0;
    public static double tolerenceRPM = 100.0; //if it is within 100 rpm then be ready
    public static double limelightangle = 25.0;
    public static double limelightheight = 12.0;
    public static double targetheight = 48.0;
    public static boolean isBusy = false;

    public static InterpLUT lut = new InterpLUT();
// this is for when hood will be added

    public Outtake(HardwareMap hwMap) {
        BottomouttakeMotor = hwMap.get(DcMotorEx.class, "outtakeMotor");
        BottomouttakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BottomouttakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BottomouttakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        flyWheelMotor = hwMap.get(DcMotorEx.class, "flyWheelMotor");
        flyWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flyWheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flyWheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0); //change later to correct one
        limelight.start();


        start = new InstantCommand(() -> {
            BottomouttakeMotor.setPower(1);
            flyWheelMotor.setPower(-1);
            isBusy = true;

        });

        reverse = new InstantCommand(() -> {
            BottomouttakeMotor.setPower(-1);
            flyWheelMotor.setPower(1);
            isBusy = true;
        });

        stop = new InstantCommand(() -> {
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

    public double getDistanceToTarget() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            double ty = result.getTy(); // Vertical offset in degrees
            double angleToGoal = limelightangle + ty;
            double distance = (targetheight - limelightheight) / Math.tan(Math.toRadians(angleToGoal));
            return distance;
        }


        return -1; // Return -1 if no valid target detected
    }

    public double getDistancetoTarger3d() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                return  Math.sqrt(Math.pow(botpose.getPosition().x, 2) +
                        Math.pow(botpose.getPosition().y, 2) +
                        Math.pow(botpose.getPosition().z, 2));
            }
        }
        return -1;
    }
    
    public double getTargetRPMforDistnace() {
        double distance = getDistanceToTarget();

        if (distance > 0) {
            return lut.get(distance);
        }
        return targetRPM;
    }

        public boolean isReadyToShoot() {
            double currentRPM = getTopRPM();
            return Math.abs(currentRPM - targetRPM) < +tolerenceRPM;

        }

        public static double getRPMPercentage () {
            return (getTopRPM() / targetRPM) * 100;
        }

        public void updateTelemetry (Telemetry telemetry){
            double currentRPM = getTopRPM();
            double flyWheelRPM = getFlywheelRPM();
            boolean ready = isReadyToShoot();
            telemetry.addData("Outtake RPM", currentRPM);
            telemetry.addData("Flywheel RPM", flyWheelRPM);
            telemetry.addData("Flywheel RPM %", getFlywheelRPM());
            telemetry.addData("Ready to Shoot", ready);
            telemetry.addData("Outtake RPM %", getRPMPercentage());
            telemetry.addData("Distance to Target", getDistanceToTarget());
            telemetry.addData("Distance to Target 3d", getDistancetoTarger3d());
            telemetry.update();
        }


    }


