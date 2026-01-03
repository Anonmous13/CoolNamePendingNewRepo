package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Blue 12 Ball Auto Near")
public class Auto12Near extends OpMode {
    private long turretStartTime = 0;
    private long lastFlingerTime = 0;
    private int currentFlinger = 0;
    private boolean isFiring = false;
    private int turretDuration = 0;
    private int flingerDelay = 0;
    private boolean firingComplete = false;

    private DcMotorEx intake;
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private DcMotor s1, s2;
    private CRServo t1, t2;
    private Servo armLeft, armRight, armBack, hood;
    private int pathState;

    private final Pose startPose = new Pose(18.504, 115.058, Math.toRadians(90));
    private final Pose shootPose = new Pose(58.596, 84.692, Math.toRadians(180));
    private final Pose pick1 = new Pose(18.504, 84.692, Math.toRadians(180));
    private final Pose pick2 = new Pose(13.997, 58.834, Math.toRadians(180));
    private final Pose pick3 = new Pose(13.759, 34.873, Math.toRadians(180));

    private PathChain shootPre, pickup1, shoot1, pickup2, shoot2, pickup3, shoot3;

    public void buildPaths(){
        shootPre = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        pickup1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, pick1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pick1.getHeading())
                .build();

        shoot1 = follower.pathBuilder()
                .addPath(new BezierLine(pick1, shootPose))
                .setLinearHeadingInterpolation(pick1.getHeading(), shootPose.getHeading())
                .build();

        pickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, new Pose(99.63756177924218, 57.172981878088976), pick2))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pick2.getHeading())
                .build();

        shoot2 = follower.pathBuilder()
                .addPath(new BezierLine(pick2, shootPose))
                .setLinearHeadingInterpolation(pick2.getHeading(), shootPose.getHeading())
                .build();

        pickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, new Pose(94.18121911037892, 38.668863261944), pick3))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pick3.getHeading())
                .build();

        shoot3 = follower.pathBuilder()
                .addPath(new BezierLine(pick3, shootPose))
                .setLinearHeadingInterpolation(pick3.getHeading(), shootPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate(){
        switch(pathState) {
            case 0:
                if(!follower.isBusy()){
                    hood.setPosition(0.4);
                    follower.followPath(shootPre, 1.0, true);
                    setPathState(1);
                }
                break;
            case 1:
                if(!follower.isBusy()){
                    startFiring(4000, 400);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy() && isFiringComplete()){
                    intake();
                    follower.followPath(pickup1, 1.0, false);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy() && isFiringComplete()){
                    follower.followPath(shoot1, 1.0, true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()){
                    startFiring(4000, 400);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy() && isFiringComplete()){
                    intake();
                    follower.followPath(pickup2, 1.0, false);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy() && isFiringComplete()){
                    follower.followPath(shoot2, 1.0, true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()){
                    startFiring(4000, 400);
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy() && isFiringComplete()){
                    intake();
                    follower.followPath(pickup3, 1.0, false);
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy() && isFiringComplete()){
                    follower.followPath(shoot3, 1.0, true);
                    setPathState(10);
                }
                break;
            case 10:
                if(!follower.isBusy()){
                    startFiring(4000, 400);
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy() && isFiringComplete()){
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void intake() {
        intake.setPower(1.0);
    }

    public void startFiring(int turretRunTimeMs, int flingerDelayMs) {
        turretStartTime = System.currentTimeMillis();
        lastFlingerTime = turretStartTime;
        currentFlinger = 0;
        isFiring = true;
        turretDuration = turretRunTimeMs;
        flingerDelay = flingerDelayMs;
        firingComplete = false;

        s1.setPower(0.4);
        s2.setPower(0.4);
    }

    public void updateFiring() {
        if (!isFiring) return;

        long currentTime = System.currentTimeMillis();
        long elapsedTime = currentTime - turretStartTime;

        if (elapsedTime >= turretDuration) {
            s1.setPower(0);
            s2.setPower(0);
            isFiring = false;
            firingComplete = true;
            return;
        }

        if (currentTime - lastFlingerTime >= flingerDelay && currentFlinger < 3) {
            if (currentFlinger == 0) {
                armLeft.setPosition(0.5);
            } else if (currentFlinger == 1) {
                armRight.setPosition(0.5);
            } else if (currentFlinger == 2) {
                armBack.setPosition(0.5);
            }

            currentFlinger++;
            lastFlingerTime = currentTime;
        }

        if (currentTime - lastFlingerTime >= 200) {
            armLeft.setPosition(1.0);
            armRight.setPosition(1.0);
            armBack.setPosition(1.0);
        }
    }

    public boolean isFiringComplete() {
        return firingComplete;
    }

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotorEx.class, "fi");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        s1 = hardwareMap.get(DcMotor.class, "s1");
        s2 = hardwareMap.get(DcMotor.class, "s2");

        armLeft = hardwareMap.get(Servo.class, "armLeft");
        armRight = hardwareMap.get(Servo.class, "armRight");
        armBack = hardwareMap.get(Servo.class, "armBack");
        hood = hardwareMap.get(Servo.class, "hood");

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void loop() {
        updateFiring();
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("firing", isFiring);
        telemetry.addData("firing complete", firingComplete);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {}

    @Override
    public void init_loop() {}
}