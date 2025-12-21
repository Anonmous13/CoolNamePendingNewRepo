package org.firstinspires.ftc.teamcode.opmode.tests;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

@Autonomous(name = "Limelight Turret Auto Track")
public class LimelightTurretAuto extends LinearOpMode {

    // ===== CHANGE THESE =====
    private static final int TAG_ID_1 = 20;
    private static final int TAG_ID_2 = 24;

    private static final double STEP = 0.03;       // Servo movement step per loop
    private static final double DEADBAND = 1.0;    // Degrees within which we stop moving
    // =======================

    Limelight3A limelight;
    Servo turretLeft;
    Servo turretRight;
    double turretPos = 0.5; // Center position

    @Override
    public void runOpMode() {

        // Initialize Hardware
        turretLeft  = hardwareMap.get(Servo.class, "turretLeft");
        turretRight = hardwareMap.get(Servo.class, "turretRight");
        limelight   = hardwareMap.get(Limelight3A.class, "limelight");

        // Reverse ONE servo so they move together
        turretRight.setDirection(Servo.Direction.REVERSE);

        // Set initial positions
        turretLeft.setPosition(turretPos);
        turretRight.setPosition(turretPos);

        // Start Limelight polling
        limelight.start();

        telemetry.addData("Status", "Initialized. Waiting for start...");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

                if (fiducials != null && !fiducials.isEmpty()) {

                    LLResultTypes.FiducialResult target = fiducials.get(0);
                    int id = target.getFiducialId();
                    double tx = target.getTargetXDegrees();

                    // Only react to specific IDs
                    if (id == TAG_ID_1 || id == TAG_ID_2) {

                        // Move turret towards target if outside deadband
                        if (tx > DEADBAND) {
                            turretPos += STEP; // Move right
                        } else if (tx < -DEADBAND) {
                            turretPos -= STEP; // Move left
                        }

                        // Clamp servo values between 0.0 and 1.0
                        turretPos = Math.max(0.0, Math.min(1.0, turretPos));

                        // Apply positions to both servos
                        turretLeft.setPosition(turretPos);
                        turretRight.setPosition(turretPos);

                        telemetry.addData("Targeting", "ID %d", id);
                        telemetry.addData("TX Error", "%.2f", tx);
                        telemetry.addData("Turret Pos", "%.3f", turretPos);

                    }
                } else {
                    telemetry.addData("Status", "No Tags Detected");
                }

            } else {
                telemetry.addData("Status", "No Valid Limelight Result");
            }

            telemetry.update();
        }

        limelight.stop();
    }
}
