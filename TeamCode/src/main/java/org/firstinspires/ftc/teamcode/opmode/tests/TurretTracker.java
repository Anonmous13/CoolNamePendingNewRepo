package org.firstinspires.ftc.teamcode.opmode.tests;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp(name = "CR Turret Tracker - ID 20")
public class TurretTracker extends LinearOpMode {

    private Limelight3A limelight;
    private CRServo turretServo;

    public double tX;

    private double smoothedTx = 0;
    private double lastTx = 0;

    // ===== CR SERVO TUNING =====
    final double kP = 0.02;          // how strongly it reacts to error
    final double kD = 0.002;         // dampens oscillation
    final double MAX_POWER = 0.35;   // cap rotation speed
    final double DEADZONE = 2.5;     // degrees Limelight error

    @Override
    public void runOpMode() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turretServo = hardwareMap.get(CRServo.class, "turret_servo");

        limelight.pipelineSwitch(8);
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();
            if (result == null || !result.isValid()) {
                turretServo.setPower(0);
                continue;
            }

            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

            boolean foundTarget = false;

            for (LLResultTypes.FiducialResult tag : tags) {

                if (tag.getFiducialId() != 20) continue;
                foundTarget = true;

                tX = result.getTx();   // horizontal error in degrees

                // Deadzone so it doesn’t jitter
                if (Math.abs(tX) < DEADZONE) {
                    turretServo.setPower(0);
                    continue;
                }

                // Smooth the signal
                double alpha = 0.2;
                smoothedTx = alpha * smoothedTx + (1.0 - alpha) * tX;

                double dTx = smoothedTx - lastTx;
                lastTx = smoothedTx;

                // PD control → power
                double power = (kP * smoothedTx) + (kD * dTx);

                // Clamp power
                power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));

                // Direction might be reversed — flip sign if needed
                turretServo.setPower(power);
            }

            // No tag → stop turret
            if (!foundTarget) {
                turretServo.setPower(0);
            }

            telemetry.addData("Limelight found target", foundTarget);
            telemetry.addData("Limelight ID:", limelight.getLatestResult());

        }
    }
}
