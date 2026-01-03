package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "Blue Auto 12 ball Far")
public class Auto12Far extends OpMode {
    private double intakePower = 1.0;
    private double drivePower = 1.0;
    private ElapsedTime turretTimer = new ElapsedTime();
    private int turretState = 0;
    private double s1Pow, s2Pow;
    private long spinupTime, flingerDelay;
    private double upPos, downPos;
    private DcMotor s1, s2, fi;
    private CRServo t1, t2;
    private Servo armLeft, armRight, armBack, hood;
    private Follower follower;
    private int pathState;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private final Pose startPose = new Pose(62.71286944090499, 8.303130148270183, Math.toRadians(180));
    private final Pose shootPose = new Pose(21.423780961556382, 12.164372067004386, Math.toRadians(180));
    private final Pose pickupPose = new Pose(20.609907698045184, 11.807937346551865, Math.toRadians(180));
    private final Pose Park = new Pose(44.28318364048856, 30.19405165180097, Math.toRadians(180));

    private Path park;
    private PathChain shootPreLoad, pickup1, shoot1, pickup2, shoot2, pickup3, shoot3;

    public void buildPaths(){
        shootPreLoad = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        pickup1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, pickupPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickupPose.getHeading())
                .build();

        shoot1 = follower.pathBuilder()
                .addPath(new BezierLine(pickupPose, shootPose))
                .setLinearHeadingInterpolation(pickupPose.getHeading(), shootPose.getHeading())
                .build();

        pickup2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, pickupPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickupPose.getHeading())
                .build();

        shoot2 = follower.pathBuilder()
                .addPath(new BezierLine(pickupPose, shootPose))
                .setLinearHeadingInterpolation(pickupPose.getHeading(), shootPose.getHeading())
                .build();

        pickup3 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, pickupPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickupPose.getHeading())
                .build();

        shoot3 = follower.pathBuilder()
                .addPath(new BezierLine(pickupPose, shootPose))
                .setLinearHeadingInterpolation(pickupPose.getHeading(), shootPose.getHeading())
                .build();

        park = new Path(new BezierLine(shootPose, Park));
        park.setLinearHeadingInterpolation(shootPose.getHeading(), Park.getHeading());
    }

    public void autonomousPathUpdate(){
        switch(pathState){
            case 0:
                if(!follower.isBusy()){
                    hood.setPosition(0.4);
                    follower.followPath(shootPreLoad, 1.0, true);
                    setPathState(1);
                }
                break;
            case 1:
                if(!follower.isBusy()){
                    startTurretSequence(0.55, 0.55, 1500, 400, 0.5, 0.0);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy() && turretState == 0){ // Wait for turret to finish
                    fi.setPower(1.0);
                    follower.followPath(pickup1, 1.0, false);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy() && turretState == 0){
                    hood.setPosition(0.4);
                    follower.followPath(shoot1, 1.0, true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()){
                    startTurretSequence(0.55, 0.55, 1500, 400, 0.5, 0.0);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy() && turretState == 0){ // Wait for turret to finish
                    fi.setPower(1.0);
                    follower.followPath(pickup2, 1.0, false);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy() && turretState == 0){
                    hood.setPosition(0.4);
                    follower.followPath(shoot2, 1.0, true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()){
                    startTurretSequence(0.55, 0.55, 1500, 400, 0.5, 0.0);
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy() && turretState == 0){ // Wait for turret to finish
                    fi.setPower(1.0);
                    follower.followPath(pickup3, 1.0, false);
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy() && turretState == 0){
                    hood.setPosition(0.4);
                    follower.followPath(shoot3, 1.0, true);
                    setPathState(10);
                }
                break;
            case 10:
                if(!follower.isBusy()){
                    startTurretSequence(0.55, 0.55, 1500, 400, 0.5, 0.0);
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy() && turretState == 0){ // Wait for turret to finish
                    follower.followPath(park, true);
                    setPathState(12);
                }
                break;
            case 12:
                if(!follower.isBusy()){
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void startTurretSequence(double s1Power, double s2Power,
                                    long flywheelSpinupMs, long flingerDelayMs,
                                    double flingerUpPos, double flingerDownPos) {
        s1Pow = s1Power;
        s2Pow = s2Power;
        spinupTime = flywheelSpinupMs;
        flingerDelay = flingerDelayMs;
        upPos = flingerUpPos;
        downPos = flingerDownPos;

        turretState = 1;
        turretTimer.reset();

        s1.setPower(s1Pow);
        s2.setPower(s2Pow);
    }

    public void updateTurretSequence() {
        double elapsed = turretTimer.milliseconds();

        switch(turretState) {
            case 1: // Flywheel spinup
                if(elapsed >= spinupTime) {
                    armLeft.setPosition(upPos);
                    turretTimer.reset();
                    turretState = 2;
                }
                break;

            case 2: // armLeft up
                if(elapsed >= flingerDelay) {
                    armLeft.setPosition(downPos);
                    armRight.setPosition(upPos);
                    turretTimer.reset();
                    turretState = 3;
                }
                break;

            case 3: // armRight up
                if(elapsed >= flingerDelay) {
                    armRight.setPosition(downPos);
                    armBack.setPosition(upPos);
                    turretTimer.reset();
                    turretState = 4;
                }
                break;

            case 4: // armBack up
                if(elapsed >= flingerDelay) {
                    armBack.setPosition(downPos);
                    s1.setPower(0);
                    s2.setPower(0);
                    turretState = 0; // Sequence complete
                }
                break;
        }
    }

    // Helper method to check if turret is done
    public boolean isTurretDone() {
        return turretState == 0;
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void loop() {
        updateTurretSequence();
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("turret state", turretState);
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