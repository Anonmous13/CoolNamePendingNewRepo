package org.firstinspires.ftc.teamcode.opmode.tele;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Mecanum + Gates + Rack AutoTop Start+End", group = "TeleOp")
public class MecanumWithGates extends LinearOpMode {


    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor frontIntake, frontintakel, backShooter;

    private DcMotor rack = null;
    private Servo leftGate, rightGate, middleGate;


    // gate positions
    private final double LEFT_UP = .3;
    private final double LEFT_DOWN = 0.2;
    private final double RIGHT_UP = 0.7;
    private final double RIGHT_DOWN = 0.2;
    private final double MID_UP = 0.8;
    private final double MID_DOWN = 0.2;


    // software rack limits
    private final int RACK_TOP = 0;       // top position
    private final int RACK_BOTTOM = -2300; // bottom position


    @Override
    public void runOpMode() {
        // ------------------- HARDWARE MAP -------------------
        frontLeft  = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft   = hardwareMap.get(DcMotor.class, "bl");
        backRight  = hardwareMap.get(DcMotor.class, "br");


        frontIntake = hardwareMap.get(DcMotor.class, "fi");
        frontintakel = hardwareMap.get(DcMotor.class, "fi2");
        backShooter = hardwareMap.get(DcMotor.class, "bs");
        rack = hardwareMap.get(DcMotor.class, "rk");


        leftGate = hardwareMap.get(Servo.class, "lg");
        rightGate = hardwareMap.get(Servo.class, "rg");
        middleGate = hardwareMap.get(Servo.class, "mg");


        // ------------------- MOTOR CONFIG -------------------
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontintakel.setDirection(DcMotor.Direction.REVERSE);
        frontIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontintakel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        rack = hardwareMap.get(DcMotor.class, "rk");
        rack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Brake so the rack holds position when joystick is released
        rack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reverse this if pushing up moves the rack down
        // rackMotor.setDirection(DcMotor.Direction.REVERSE);



        waitForStart();

        // FTC joystick: up is NEGATIVE, so invert it
        if (gamepad2.right_bumper) {
            rack.setPower(1);
        } else if(gamepad2.left_bumper) {
            rack.setPower(-1);
        } else {
            rack.setPower(0);
        }


        while (opModeIsActive()) {


            // ----- DRIVETRAIN -----
            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;


            double frontLeftPower = y + x + rx;
            double backLeftPower = y - x + rx;
            double frontRightPower = y - x - rx;
            double backRightPower = y + x - rx;


            double max = Math.max(Math.abs(frontLeftPower),
                    Math.max(Math.abs(backLeftPower),
                            Math.max(Math.abs(frontRightPower),
                                    Math.abs(backRightPower))));
            if (max > 1.0) {
                frontLeftPower /= max;
                backLeftPower /= max;
                frontRightPower /= max;
                backRightPower /= max;
            }


            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);


            // ----- INTAKE / SHOOTER -----
            if (gamepad2.a) {
                frontIntake.setPower(1.0);
                frontintakel.setPower(-1.0);
                backShooter.setPower(0);
            } else if (gamepad2.b) {
                frontIntake.setPower(1.0);
                frontintakel.setPower(-1.0);
                backShooter.setPower(1.0);
            } else if (gamepad2.x) {
                frontIntake.setPower(-1.0);
                frontintakel.setPower(1.0);
                backShooter.setPower(0);
            } else if (gamepad2.y) {
                frontIntake.setPower(-1.0);
                frontintakel.setPower(1.0);
                backShooter.setPower(-1.0);
            } else {
                frontIntake.setPower(0);
                frontintakel.setPower(0);
                backShooter.setPower(0);
            }


            // ----- GATES -----
            if (gamepad2.dpad_left)  leftGate.setPosition(LEFT_UP);
            if (gamepad2.dpad_right) rightGate.setPosition(RIGHT_UP);
            if (gamepad2.dpad_up)    middleGate.setPosition(MID_UP);
            if (gamepad2.dpad_down) {
                middleGate.setPosition(0.085);
                rightGate.setPosition(0.55);
                leftGate.setPosition(0.73);
            }



            // ----- TELEMETRY -----
            // Send the motor power and joystick position to the Driver Station for debugging
            telemetry.addData("Status", "Running");
            telemetry.addData("Motor Power", rack.getPower());
            telemetry.addData("Rack Top Limit", RACK_TOP);
            telemetry.addData("Rack Bottom Limit", RACK_BOTTOM);
            telemetry.addData("LeftGate", leftGate.getPosition());
            telemetry.addData("RightGate", rightGate.getPosition());
            telemetry.addData("MidGate", middleGate.getPosition());
            telemetry.addData("FrontIntake", frontIntake.getPower());
            telemetry.addData("BackShooter", backShooter.getPower());
            telemetry.addData("FrontLeft", frontLeft.getPower());
            telemetry.addData("BackLeft", backLeft.getPower());
            telemetry.addData("FrontRight", frontRight.getPower());
            telemetry.addData("BackRight", backRight.getPower());
            telemetry.update();
        }


        rack.setPower(0);
        rack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
