package org.firstinspires.ftc.teamcode.opmode.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TeleOp Turret + Shooter")
public class TeleOpTurretBot extends OpMode {

    private double turretPower = 0.55;

    private double endGameStart;
    private boolean isEndGame;

    // Drive motors
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Shooter motors
    private DcMotor s1, s2;

    // Turret servos (CRServos)
    private CRServo t1, t2, angleServo;

    // Flinger servos (NORMAL servos)
    private Servo f1, f2;

    @Override
    public void init() {

        // Drive motors
        frontLeft  = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft   = hardwareMap.get(DcMotor.class, "bl");
        backRight  = hardwareMap.get(DcMotor.class, "br");

        // Shooter motors
        s1 = hardwareMap.get(DcMotor.class, "s1");
        s2 = hardwareMap.get(DcMotor.class, "s2");

        // Turret CRServos
        t1 = hardwareMap.get(CRServo.class, "t1");
        t2 = hardwareMap.get(CRServo.class, "t2");

        // Angle servo
        angleServo = hardwareMap.get(CRServo.class, "as");

        // Flinger servos
        f1 = hardwareMap.get(Servo.class, "f1");
        f2 = hardwareMap.get(Servo.class, "f2");

        // Motor directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        s1.setDirection(DcMotor.Direction.REVERSE);
        s2.setDirection(DcMotor.Direction.REVERSE);

        // Brake when stopped
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        s1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        s2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Start flingers DOWN
        f1.setPosition(0.0);
        f2.setPosition(0.0);
    }

    @Override
    public void loop() {

        // Drive commands
        double y  = -gamepad1.left_stick_y;
        double x  =  gamepad1.left_stick_x;
        double rx =  gamepad1.right_stick_x;

        // Equations for the moving
        double fl = y + x + rx;
        double fr = y - x - rx;
        double bl = y - x + rx;
        double br = y + x - rx;

        // Clips the powers between 1, -1
        fl = Range.clip(fl, -1.0, 1.0);
        fr = Range.clip(fr, -1.0, 1.0);
        bl = Range.clip(bl, -1.0, 1.0);
        br = Range.clip(br, -1.0, 1.0);

        // Sets the motor powers when driving
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);

        // Makes the shooter for turret run
        if (gamepad2.a) {
            s1.setPower(0.55);
            s2.setPower(0.55);
        } else {
            s1.setPower(0);
            s2.setPower(0);
        }

        // Makes the shooter stop
        if (gamepad2.x) {
            s1.setPower(0);
            s2.setPower(0);
        }

        // Makes the flingers move
        if (gamepad2.dpad_up) {
            f1.setPosition(1.0);
            f2.setPosition(1.0);
        } else if (gamepad2.dpad_down) {
            f1.setPosition(0.0);
            f2.setPosition(0.0);
        }

        // Angle for the shooter
        if (gamepad2.right_bumper) {
            angleServo.setPower(1);
        } else if (gamepad2.left_bumper) {
            angleServo.setPower(-1);
        } else {
            angleServo.setPower(0);
        }

        if (getRuntime() >= endGameStart && !isEndGame) {
            // Vibrate both controllers
            gamepad1.rumbleBlips(3);
            gamepad2.rumbleBlips(3);

            // Mark endgame as triggered
            isEndGame = true;
        }


        // Makes it so we can move the turret with joystick
        turretPower = Range.clip(gamepad1.right_stick_x, -0.6, 0.6);

        t1.setPower(turretPower);
        t2.setPower(-turretPower);

        // Telemetry
        telemetry.addData("fl", frontLeft.getPower());
        telemetry.addData("fr", frontRight.getPower());
        telemetry.addData("bl", backLeft.getPower());
        telemetry.addData("br", backRight.getPower());
        telemetry.addData("flinger 1 position", f1.getPosition());
        telemetry.addData("flinger 2 position", f2.getPosition());
        telemetry.addData("shooter 1 power", s1.getPower());
        telemetry.addData("shooter 2 power", s2.getPower());
        telemetry.addData("Turret Power", turretPower);
        telemetry.addData("Angle Servo", angleServo.getPower());
        telemetry.addData("Is Endgame happening?", isEndGame);
        telemetry.update();
    }

    @Override
    public void start() {
        endGameStart = getRuntime() + 120;
    }
}
