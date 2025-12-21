package org.firstinspires.ftc.teamcode.opmode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Turret 2 Servo Joystick")
public class TurretTwoServoJoystick extends LinearOpMode {

    Servo turretLeft;
    Servo turretRight;

    double turretPos = 0.5; // center position

    @Override
    public void runOpMode() {

        turretLeft  = hardwareMap.get(Servo.class, "turretLeft");
        turretRight = hardwareMap.get(Servo.class, "turretRight");

        // Reverse ONE servo so they move the same direction mechanically
        turretRight.setDirection(Servo.Direction.REVERSE);

        // Start centered
        turretLeft.setPosition(turretPos);
        turretRight.setPosition(turretPos);

        waitForStart();

        while (opModeIsActive()) {

            // Left stick X = left/right
            double stickX = gamepad1.left_stick_x;

            // Adjust turret position
            turretPos += stickX * 0.01; // CHANGE SPEED HERE

            // Keep servos safe
            turretPos = Math.max(0.0, Math.min(1.0, turretPos));

            // Move BOTH servos
            turretLeft.setPosition(turretPos);
            turretRight.setPosition(turretPos);

            telemetry.addData("Stick X", stickX);
            telemetry.addData("Turret Pos", turretPos);
            telemetry.update();
        }
    }
}
