package org.firstinspires.ftc.teamcode.opmode.tests;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "LimelightServoTracker_20_24", group = "Test")
public class LimelightServoTracker_20_24 extends LinearOpMode {

    Limelight3A limelight;
    Servo limelightServo;

    // ===== SERVO SAFETY (WIRE PROTECTION) =====
    final double CENTER = 0.5;
    final double MIN = 0.25;
    final double MAX = 0.75;
    final double STEP = 0.002;

    double servoPos = CENTER;

    // ===== TRACKING =====
    final double DEADZONE = 1.0;

    @Override
    public void runOpMode() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelightServo = hardwareMap.get(Servo.class, "limelightServo");

        limelightServo.setPosition(CENTER);

        waitForStart();

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                double tx = result.getTx();

                // ===== TRACK TARGET =====
                if (tx > DEADZONE) {
                    servoPos -= STEP;
                } else if (tx < -DEADZONE) {
                    servoPos += STEP;
                }

            } else {
                // ===== TARGET LOST → RECENTER =====
                servoPos += (CENTER - servoPos) * 0.05;
            }

            // ===== HARD CLAMP =====
            servoPos = Math.max(MIN, Math.min(MAX, servoPos));
            limelightServo.setPosition(servoPos);

            telemetry.addData("Servo Pos", servoPos);
            telemetry.addData("Has Target", result != null && result.isValid());
            telemetry.update();
        }
    }
}
