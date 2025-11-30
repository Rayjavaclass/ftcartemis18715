package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Turret Test", group="Test")
public class TurretTestOpMode extends LinearOpMode {

    private Servo turretServo;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize servo (must match config name in RC app)
        turretServo = hardwareMap.get(Servo.class, "turretServo");

        // Start at center
        turretServo.setPosition(0.5);

        telemetry.addLine("Turret Test Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Gamepad controls
            if (gamepad1.triangle) {
                turretServo.setPosition(0.0); // left
                telemetry.addLine("Turret LEFT");
            } else if (gamepad1.circle) {
                turretServo.setPosition(1.0); // right
                telemetry.addLine("Turret RIGHT");
            } else if (gamepad1.cross) {
                turretServo.setPosition(0.5); // center
                telemetry.addLine("Turret CENTER");
            }

            // Show current position
            telemetry.addData("Turret Position", turretServo.getPosition());
            telemetry.update();
        }
    }
}
