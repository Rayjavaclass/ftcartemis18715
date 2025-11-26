package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.pedropathing.geometry.Pose;

public class DrivetrainHelper {

    private DcMotor frontLeft, backLeft, frontRight, backRight;
    private Limelight3A limelight;
    private Telemetry telemetry;

    private Pose targetPosition = new Pose(0, 72, 0);
    private double lastHeadingError = 0;
    private double kP = 0.02;
    private double kD = 0.01;

    public enum DriveMode {
        DRIVER_CONTROL,
        LIMELIGHT_ALIGNMENT,
        HEADING_LOCK
    }

    private DriveMode currentMode = DriveMode.DRIVER_CONTROL;

    public DrivetrainHelper(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Initialize follower pose
        follower.setPose(new Pose(0, 0, 0));
    }

    public void update(Gamepad gamepad1) {
        // Update follower each loop
        follower.update();

        // Handle mode toggles
        if (gamepad1.a) {
            currentMode = DriveMode.LIMELIGHT_ALIGNMENT;
        } else if (gamepad1.b) {
            currentMode = DriveMode.HEADING_LOCK;
        } else if (gamepad1.y) {
            currentMode = DriveMode.DRIVER_CONTROL;
        }

        // Execute appropriate drive mode
        switch (currentMode) {
            case LIMELIGHT_ALIGNMENT:
                driveWithLimelightAlignment(gamepad1);
                telemetry.addData("Drive Mode", "Limelight alignment");
                break;
            case HEADING_LOCK:
                driveWithHeadingLock();
                telemetry.addData("Drive Mode", "Heading lock");
                break;
            case DRIVER_CONTROL:
            default:
                driveWithDriverControl(gamepad1);
                telemetry.addData("Drive Mode", "Driver control");
                break;
        }
    }

    private void driveWithDriverControl(Gamepad gamepad1) {
        double drive = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        Pose currentPose = follower.getPose();
        if (currentPose == null) {
            telemetry.addLine("️ Pose is null!");
            return;
        }

        double botHeading = currentPose.getHeading();

        // Field-centric drive
        double rotX = strafe * Math.cos(-botHeading) - drive * Math.sin(-botHeading);
        double rotY = strafe * Math.sin(-botHeading) + drive * Math.cos(-botHeading);

        double fl = rotY + rotX + turn;
        double bl = rotY - rotX + turn;
        double fr = rotY - rotX - turn;
        double br = rotY + rotX - turn;

        // Normalize powers
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(bl)),
                Math.max(Math.abs(fr), Math.abs(br))));

        frontLeft.setPower(fl / max);
        backLeft.setPower(bl / max);
        frontRight.setPower(fr / max);
        backRight.setPower(br / max);

        telemetry.addData("Pose X", currentPose.getX());
        telemetry.addData("Pose Y", currentPose.getY());
        telemetry.addData("Heading", currentPose.getHeading());
    }

    private void driveWithLimelightAlignment(Gamepad gamepad1) {
        if (limelight == null) {
            telemetry.addData("Limelight", "Not available");
            return;
        }

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            double tx = result.getTx();
            double ty = result.getTy();

            double headingError = -tx;
            double headingErrorDerivative = headingError - lastHeadingError;
            headingErrorDerivative = Math.max(-10, Math.min(10, headingErrorDerivative));
            lastHeadingError = headingError;

            double rotationPower = (headingError * 0.03) + (headingErrorDerivative * kD);
            rotationPower = Math.max(-1, Math.min(1, rotationPower));

            double forwardPower = -ty * 0.02;

            double drive = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;

            Pose currentPose = follower.getPose();
            if (currentPose == null) {
                telemetry.addLine("⚠️ Pose is null!");
                return;
            }

            double botHeading = currentPose.getHeading();

            double rotX = strafe * Math.cos(-botHeading) - drive * Math.sin(-botHeading);
            double rotY = strafe * Math.sin(-botHeading) + drive * Math.cos(-botHeading);

            double fl = rotY + rotX + rotationPower + forwardPower;
            double bl = rotY - rotX + rotationPower + forwardPower;
            double fr = rotY - rotX - rotationPower + forwardPower;
            double br = rotY + rotX - rotationPower + forwardPower;

            double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(bl)),
                    Math.max(Math.abs(fr), Math.abs(br))));

            frontLeft.setPower(fl / max);
            backLeft.setPower(bl / max);
            frontRight.setPower(fr / max);
            backRight.setPower(br / max);

            telemetry.addData("Limelight Tx", tx);
            telemetry.addData("Limelight Ty", ty);
            telemetry.addData("Rotation Power", rotationPower);
            telemetry.addData("Forward Power", forwardPower);
            telemetry.addData("Pose X", currentPose.getX());
            telemetry.addData("Pose Y", currentPose.getY());
            telemetry.addData("Heading", currentPose.getHeading());
        } else {
            telemetry.addData("Limelight", "No target");
        }
    }

    private void driveWithHeadingLock() {
        Pose currentPose = follower.getPose();
        if (currentPose == null || targetPosition == null) {
            telemetry.addLine("⚠️ Pose or target is null!");
            return;
        }

        double deltaX = targetPosition.getX() - currentPose.getX();
        double deltaY = targetPosition.getY() - currentPose.getY();
        double targetHeading = Math.atan2(deltaY, deltaX);

        double headingError = normalizeAngle(targetHeading - currentPose.getHeading());

        double headingErrorDerivative = normalizeAngle(headingError - lastHeadingError);
        headingErrorDerivative = Math.max(-1, Math.min(1, headingErrorDerivative));
        lastHeadingError = headingError;

        double rotationPower = (headingError * kP) + (headingErrorDerivative * kD);
        rotationPower = Math.max(-1, Math.min(1, rotationPower));

        frontLeft.setPower(rotationPower);
        backLeft.setPower(rotationPower);
        frontRight.setPower(-rotationPower);
        backRight.setPower(-rotationPower);

        telemetry.addData("Heading Error (rad)", headingError);
        telemetry.addData("Rotation Power", rotationPower);
        telemetry.addData("Target X", targetPosition.getX());
        telemetry.addData("Target Y", targetPosition.getY());
        telemetry.addData("Current X", currentPose.getX());
        telemetry.addData("Current Y", currentPose.getY());
        telemetry.addData("Current Heading", currentPose.getHeading());
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    // Setter methods for configuration
    public void setTargetPosition(Pose target) {
        this.targetPosition = target;
    }

    public void setKP(double kP) {
        this.kP = kP;
    }

    public void setKD(double kD) {
        this.kD = kD;
    }

    public DriveMode getCurrentMode() {
        return currentMode;
    }

    public void setCurrentMode(DriveMode mode) {
        this.currentMode = mode;
    }

    public Pose getCurrentPose() {
        return follower.getPose();
    }
}
