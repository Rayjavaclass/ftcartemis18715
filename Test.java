package org.firstinspires.ftc.teamcode.teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Drawing;
import org.firstinspires.ftc.teamcode.Robot;

import dev.nextftc.core.commands.CommandManager;


@TeleOp(name="BLUE TELEOP", group="TeleOp")
public class Test extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, Alliance.BLUE);
        CommandManager.INSTANCE.cancelAll();

        waitForStart();
        while(opModeIsActive()) {
            robot.periodic();

            telemetry.addData("[ODO] Pose x", robot.follower.getPose().getX());
            telemetry.addData("[ODO] Pose y", robot.follower.getPose().getY());
            telemetry.addData("[ODO] Heading", robot.follower.getPose().getHeading());



            telemetry.addData("[SHOOTER] RPM", robot.outtake.getTopTPS());
            telemetry.addData("[HOOD] Tick: ", robot.outtake.hood.getPosition());
            telemetry.update();

            Drawing.init();
            Drawing.drawRobot(robot.follower.getPose());
            Drawing.drawPoseHistory(robot.follower.getPoseHistory());
            Drawing.sendPacket();
        }
    }
}
