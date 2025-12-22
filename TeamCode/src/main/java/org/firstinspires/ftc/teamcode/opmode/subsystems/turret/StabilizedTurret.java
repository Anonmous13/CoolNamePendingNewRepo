package org.firstinspires.ftc.teamcode.opmode.subsystems.turret;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp(name = "CR Turret Fix - ID 20")
public class StabilizedTurret extends LinearOpMode {

    private Limelight3A limelight;
    private Servo turretServo;
    public double distance;
    public double pitch;
    public double yaw;
    public double roll;
    public double tX;
    public double tY;

    private double smoothedTx;
    private double lastTx;

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
            if (result == null || !result.isValid()) {
                return;
            }

            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

            for (LLResultTypes.FiducialResult tag : tags) {

                if (tag.getFiducialId() != 20) {
                    continue;
                }

                Pose3D pose = tag.getRobotPoseTargetSpace();

                distance = Math.sqrt(
                        pose.getPosition().x * pose.getPosition().x +
                                pose.getPosition().y * pose.getPosition().y +
                                pose.getPosition().z * pose.getPosition().z
                ) * (64.0 / 12.0);

                pitch = pose.getOrientation().getPitch(AngleUnit.DEGREES);
                yaw   = pose.getOrientation().getYaw(AngleUnit.DEGREES);
                roll  = pose.getOrientation().getRoll(AngleUnit.DEGREES);

                tX = result.getTx();
                tY = result.getTy();

                double rawTx = tX;
                if (Math.abs(rawTx) < 2.5) {
                    return;
                }

                boolean turning = Math.abs(gamepad1.right_stick_x) > 0.05;

                double alpha   = turning ? 0.16  : 0.20;
                double kP      = turning ? 0.0028 : 0.0014;
                double kD      = turning ? 0.0002 : 0.0008;
                double maxStep = turning ? 0.035  : 0.020;

                smoothedTx = alpha * smoothedTx + (1.0 - alpha) * rawTx;

                double dTx = smoothedTx - lastTx;
                lastTx = smoothedTx;

                double correction = (kP * smoothedTx) + (kD * dTx);

                double current = turretServo.getPosition();
                double target = current + correction;

                double delta = target - current;
                if (Math.abs(delta) > maxStep) {
                    delta = Math.copySign(maxStep, delta);
                }

                double pos = Math.max(0.0, Math.min(1.0, current + delta));
                turretServo.setPosition(pos);

                return;
            }
    }
}}