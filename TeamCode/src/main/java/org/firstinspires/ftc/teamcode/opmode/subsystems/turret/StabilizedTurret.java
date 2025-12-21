package org.firstinspires.ftc.teamcode.opmode.subsystems.turret;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.List;

@TeleOp(name = "CR Turret Fix - ID 20")
public class StabilizedTurret extends LinearOpMode {

    private Limelight3A limelight;
    private Servo turretServo;

    // --- TUNING FOR CR SERVO ---
    // If it spins too fast/wildly, lower this (e.g., 0.01).
    // If it's too weak to move, raise it (e.g., 0.05).
    final double Kp = 0.02;
    final double STOP_POWER = 0.5; // CR Servos stop at 0.5

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turretServo = hardwareMap.get(Servo.class, "turret_servo");

        limelight.pipelineSwitch(0);
        limelight.start();

        // Safety: Ensure it is stopped before we start
        turretServo.setPosition(STOP_POWER);

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

                boolean found = false;
                for (LLResultTypes.FiducialResult fr : fiducials) {
                    if (fr.getFiducialId() == 20) {
                        found = true;
                        double tx = fr.getTargetXDegrees();

                        // For a CR Servo, we calculate SPEED.
                        // If tx is positive, we need to spin one way.
                        // If tx is negative, we spin the other way.
                        double turretSpeed = (tx * Kp);

                        // Apply the speed to the STOP_POWER (0.5)
                        // Try (STOP_POWER + turretSpeed) if it spins the wrong way
                        double finalCommand = STOP_POWER - turretSpeed;

                        // Clamp to safe power levels
                        finalCommand = Math.max(0.0, Math.min(1.0, finalCommand));

                        turretServo.setPosition(finalCommand);

                        telemetry.addData("TX Error", tx);
                        telemetry.addData("Servo Power", finalCommand);
                    }
                }

                // If we see tags but NOT ID 20, stop spinning
                if (!found) {
                    turretServo.setPosition(STOP_POWER);
                }

            } else {
                // STOP spinning if we lose sight of everything
                turretServo.setPosition(STOP_POWER);
                telemetry.addData("Status", "No Target - Stopped");
            }
            telemetry.update();
        }
        turretServo.setPosition(STOP_POWER); // Final safety stop
    }
}