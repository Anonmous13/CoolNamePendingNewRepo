package org.firstinspires.ftc.teamcode.opmode.subsystems.turret;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name="Fix: Smooth Field Align Tag 20", group="Vision")
public class FixedFieldAlign extends LinearOpMode {
    private Limelight3A limelight;
    private Servo turretServo;

    // Fixed Tag 20 Field Coordinates (Meters)
    private final double TAG_20_X = 1.8;
    private final double TAG_20_Y = 0.0;

    private double currentPos = 0.5; // Start centered
    // Adjust this: Higher = faster snap, Lower = smoother tracking
    private final double Kp = 0.002;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turretServo = hardwareMap.get(Servo.class, "turretServo");

        limelight.start();
        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.getBotpose() != null) {
                Pose3D botpose = result.getBotpose();
                double robotX = botpose.getPosition().x;
                double robotY = botpose.getPosition().y;
                double robotHeading = botpose.getOrientation().getYaw(AngleUnit.DEGREES);

                // Calculate Field-Relative Angle to Tag
                double angleToTag = Math.toDegrees(Math.atan2(TAG_20_Y - robotY, TAG_20_X - robotX));

                // Calculate Turret-Relative Error
                double error = angleToTag - robotHeading;

                // Normalize error to -180 to 180 degrees
                while (error > 180) error -= 360;
                while (error < -180) error += 360;

                // NUDGE the servo instead of jumping.
                // If it still turns AWAY, change the '-' to a '+'
                currentPos -= (error * Kp);

                // Clamp to physical servo limits
                currentPos = Math.max(0.1, Math.min(0.9, currentPos));
                turretServo.setPosition(currentPos);

                telemetry.addData("Error", "%.2f deg", error);
            } else {
                telemetry.addData("Status", "No Botpose - Ensure Map is loaded");
            }
            telemetry.addData("Servo Pos", currentPos);
            telemetry.update();
        }
    }
}
