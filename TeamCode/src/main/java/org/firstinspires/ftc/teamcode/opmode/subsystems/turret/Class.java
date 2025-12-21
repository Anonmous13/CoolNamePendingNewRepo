package org.firstinspires.ftc.teamcode.opmode.subsystems.turret;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Simple Servo Manual", group = "TeleOp")
public class Class extends LinearOpMode {

    private Servo turretServo;

    @Override
    public void runOpMode() {
        // Must match the name in your robot configuration on the driver hub
        turretServo = hardwareMap.get(Servo.class, "turret_servo");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // If Button A on Gamepad 2 is pressed
            if (gamepad2.a) {
                turretServo.setPosition(0.5);
                telemetry.addData("Action", "Moving to 0.5");
            }

            // If Button B on Gamepad 2 is pressed (for testing movement)
            else if (gamepad2.b) {
                turretServo.setPosition(1.0);
                telemetry.addData("Action", "Moving to 1.0");
            }

            telemetry.addData("Current Servo Pos", turretServo.getPosition());
            telemetry.update();
        }
    }
}