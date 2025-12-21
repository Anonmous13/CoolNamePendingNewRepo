package org.firstinspires.ftc.teamcode.opmode.tele;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

@TeleOp(name = "Mecanum + Limelight + Joystick Turret", group = "TeleOp")
public class  turretTeleOp extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor frontIntake, frontintakel, backShooter, rack;
    private Servo leftGate, rightGate, middleGate;

    // --- LIMELIGHT & SINGLE SERVO ADDITIONS ---
    private Limelight3A limelight;
    private Servo turretServo;
    private double turretPos = 0.5;
    private final double AUTO_STEP = 0.005;  // Speed for auto-tracking
    private final double MANUAL_STEP = 0.015; // Speed for joystick control
    private final double DEADBAND = 1.0;     // Error degrees to ignore
    // ------------------------------------------

    private final double LEFT_UP = .3, LEFT_DOWN = 0.2;
    private final double RIGHT_UP = 0.7, RIGHT_DOWN = 0.2;
    private final double MID_UP = 0.8, MID_DOWN = 0.2;

    @Override
    public void runOpMode() {
        // Hardware Mapping
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");
        frontIntake = hardwareMap.get(DcMotor.class, "fi");
        frontintakel = hardwareMap.get(DcMotor.class, "fi2");
        backShooter = hardwareMap.get(DcMotor.class, "bs");
        rack = hardwareMap.get(DcMotor.class, "rk");
        leftGate = hardwareMap.get(Servo.class, "lg");
        rightGate = hardwareMap.get(Servo.class, "rg");
        middleGate = hardwareMap.get(Servo.class, "mg");

        // --- LIMELIGHT & SINGLE SERVO INIT ---
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turretServo = hardwareMap.get(Servo.class, "turretServo");

        limelight.start();
        turretServo.setPosition(turretPos);

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontintakel.setDirection(DcMotor.Direction.REVERSE);

        rack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {

            // ----- DRIVETRAIN (Using Right Stick for strafing now to free Left Stick) -----
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.right_stick_x; // Moved strafing to right stick
            double rx = gamepad1.right_trigger - gamepad1.left_trigger; // Using triggers for rotation?
            // OR keep your original mapping, but be aware Left Stick X now controls Turret.

            double frontLeftPower = y + x + rx;
            double backLeftPower = y - x + rx;
            double frontRightPower = y - x - rx;
            double backRightPower = y + x - rx;

            double max = Math.max(1.0, Math.max(Math.abs(frontLeftPower),
                    Math.max(Math.abs(backLeftPower),
                            Math.max(Math.abs(frontRightPower), Math.abs(backRightPower)))));

            frontLeft.setPower(frontLeftPower / max);
            backLeft.setPower(backLeftPower / max);
            frontRight.setPower(frontRightPower / max);
            backRight.setPower(backRightPower / max);

            // ----- TURRET CONTROL (JOYSTICK + AUTO) -----
            double turretManualInput = gamepad1.left_stick_x;

            if (Math.abs(turretManualInput) > 0.1) {
                // MANUAL CONTROL: Joystick moves the turret
                // If stick is right (positive), increase pos. If left (negative), decrease.
                turretPos += turretManualInput * MANUAL_STEP;
            } else {
                // AUTO CONTROL: Limelight takes over if joystick is centered
                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                    if (!fiducials.isEmpty()) {
                        LLResultTypes.FiducialResult target = fiducials.get(0);
                        int id = target.getFiducialId();
                        double tx = target.getTargetXDegrees();

                        if (id == 20 || id == 24) {
                            if (tx > DEADBAND) turretPos -= AUTO_STEP;
                            else if (tx < -DEADBAND) turretPos += AUTO_STEP;
                        }
                    }
                }
            }

            // Final safety constraints
            turretPos = Math.max(0.0, Math.min(1.0, turretPos));
            turretServo.setPosition(turretPos);

            // ----- RACK -----
            if (gamepad2.right_bumper) rack.setPower(1.0);
            else if(gamepad2.left_bumper) rack.setPower(-1.0);
            else rack.setPower(0);

            // ----- INTAKE / SHOOTER -----
            if (gamepad2.a) {
                frontIntake.setPower(1.0); frontintakel.setPower(-1.0);
            } else if (gamepad2.b) {
                frontIntake.setPower(1.0); frontintakel.setPower(-1.0); backShooter.setPower(1.0);
            } else if (gamepad2.x) {
                frontIntake.setPower(-1.0); frontintakel.setPower(1.0);
            } else {
                frontIntake.setPower(0); frontintakel.setPower(0); backShooter.setPower(0);
            }

            // ----- GATES -----
            if (gamepad2.dpad_left)  leftGate.setPosition(LEFT_UP);
            if (gamepad2.dpad_right) rightGate.setPosition(RIGHT_UP);
            if (gamepad2.dpad_up)    middleGate.setPosition(MID_UP);
            if (gamepad2.dpad_down) {
                middleGate.setPosition(0.085); rightGate.setPosition(0.55); leftGate.setPosition(0.73);
            }

            telemetry.addData("Turret Pos", "%.3f", turretPos);
            telemetry.addData("Mode", Math.abs(turretManualInput) > 0.1 ? "MANUAL" : "AUTO/IDLE");
            telemetry.update();
        }
        limelight.stop();
    }
}