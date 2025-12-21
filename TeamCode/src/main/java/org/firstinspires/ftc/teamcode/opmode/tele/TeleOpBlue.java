package org.firstinspires.ftc.teamcode.opmode.tele;

import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@TeleOp()
public class TeleOpBlue extends LinearOpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private Follower follower;
    private DcMotor frontIntake, frontintakel, backShooter;

    private DcMotor rack = null;
    private Servo leftGate, rightGate, middleGate;
    private PathChain goToShoot, humanP, goPushOverflow;
    private final Pose shootPose = new Pose(22.774, 123.124, Math.toRadians(323));
    private final Pose ballOverflow = new Pose(14.234, 70.221, Math.toRadians(450));
    private final Pose goToHumanPlayer = new Pose(114.109, 13.285, Math.toRadians(0));

    private enum MotorState{
        home,
        intake,
        shoot,
    }

    MotorState currentState = MotorState.home;

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

    public void buildPaths(){
        goToShoot = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), shootPose))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), shootPose.getHeading())
                .build();
        humanP = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), goToHumanPlayer))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), goToHumanPlayer.getHeading())
                .build();
        goPushOverflow = follower.pathBuilder()
                .addPath(new BezierCurve(follower.getPose(), new Pose(80,73), ballOverflow))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), ballOverflow.getHeading())
                .addPath(new BezierCurve(follower.getPose(), new Pose(80,73), ballOverflow))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), ballOverflow.getHeading())
                .build();
    }




    @Override
    public void runOpMode() throws InterruptedException {
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


        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontintakel.setDirection(DcMotor.Direction.REVERSE);
        frontIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontintakel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        rack = hardwareMap.get(DcMotor.class, "rk");
        rack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reverse this if pushing up moves the rack down
        // rackMotor.setDirection(DcMotor.Direction.REVERSE);


        if (isStopRequested()) return;;

        waitForStart();

        switch (currentState) {

        }

        while(opModeIsActive()){
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

            if(gamepad1.a) {
                follower.followPath(goPushOverflow, 1.0, true);
            }

            if (gamepad1.b){
                follower.followPath(humanP, 1.0, true);
            }
            if (gamepad1.x){
                follower.followPath(goToShoot, 1.0, true);
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
            telemetry.addData("Position", follower.getPose());
            telemetry.addData("Heading", follower.getHeading());
            telemetry.update();
        }


        rack.setPower(0);
        rack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
