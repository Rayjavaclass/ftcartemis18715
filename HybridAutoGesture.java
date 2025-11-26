package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import org.java_websocket.server.WebSocketServer;
import org.java_websocket.WebSocket;
import org.java_websocket.handshake.ClientHandshake;

import java.net.InetSocketAddress;

@Autonomous(name="Hybrid Auto Gesture", group="Autonomous")
public class HybridAutoGesture extends LinearOpMode {

    private Servo posterServo;

    @Override
    public void runOpMode() {
        posterServo = hardwareMap.get(Servo.class, "posterServo");

        // Initialize servo to neutral
        posterServo.setPosition(0.5);

        // Start WebSocket server on port 8887
        WebSocketServer server = new WebSocketServer(new InetSocketAddress(8887)) {
            @Override
            public void onOpen(WebSocket conn, ClientHandshake handshake) {
                conn.send("Connected to Control Hub");
            }

            @Override
            public void onMessage(WebSocket conn, String message) {
                if (message.equals("LEFT")) {
                    posterServo.setPosition(0.0);
                } else if (message.equals("RIGHT")) {
                    posterServo.setPosition(1.0);
                } else if (message.equals("STOP")) {
                    posterServo.setPosition(0.5);
                }
                conn.send("received message: " + message);
                telemetry.addData("Gesture Command", message);
                telemetry.update();
            }

            @Override
            public void onClose(WebSocket conn, int code, String reason, boolean remote) {}

            @Override
            public void onError(WebSocket conn, Exception ex) {
                telemetry.addData("WebSocket Error", ex.getMessage());
                telemetry.update();
            }

            @Override
            public void onStart() {

            }
        };

        server.start();

        waitForStart();

        // Keep OpMode alive while server runs
        while (opModeIsActive()) {
            idle();
        }

        try {
            server.stop();
        } catch (Exception e) {
            telemetry.addData("Server Stop Error", e.getMessage());
            telemetry.update();
        }
    }
}
