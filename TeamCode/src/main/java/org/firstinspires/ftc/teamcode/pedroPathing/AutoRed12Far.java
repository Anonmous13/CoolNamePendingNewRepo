package org.firstinspires.ftc.teamcode.pedroPathing;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AutoRed12Far extends OpMode {

    private double intakePower = 1.0;
    private double drivePower = 1.0;
    private ElapsedTime turretTimer = new ElapsedTime();
    private int turretState = 0;
    private double s1Pow, s2Pow;
    private long spinupTime, flingerDelay;
    private double upPos, downPos;
    private Follower follower;
    private DcMotor s1, s2;
    private DcMotor fi;
    private Servo armLeft, armBack, armRight;
    private CRServo t1, t2;

    private int pathState;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private final Pose startPose = new Pose(79.85940267893218, 24.994064083040694, Math.toRadians(0));
    private final Pose shootPose = new Pose(122.42176088510426, 12.65546819156538, Math.toRadians(0));
    private final Pose pickupPose = new Pose(122.42176088510426, 12.65546819156538, Math.toRadians(0));
    private final Pose park = new Pose(96.83168101438915, 35.29704447276004, Math.toRadians(0));
    private Path Park;
    private PathChain shootPreLoad, pick1, shoot1, pick2, shoot2, pick3, shoot3;

    public void buildPaths(){
        shootPreLoad = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        pick1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, pickupPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickupPose.getHeading())
                .build();

        shoot1 = follower.pathBuilder()
                .addPath(new BezierLine(pickupPose, shootPose))
                .setLinearHeadingInterpolation(pickupPose.getHeading(), shootPose.getHeading())
                .build();

        pick2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, pickupPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickupPose.getHeading())
                .build();

        shoot2 = follower.pathBuilder()
                .addPath(new BezierLine(pickupPose, shootPose))
                .setLinearHeadingInterpolation(pickupPose.getHeading(), shootPose.getHeading())
                .build();

        pick3 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, pickupPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickupPose.getHeading())
                .build();

        shoot3 = follower.pathBuilder()
                .addPath(new BezierLine(pickupPose, shootPose))
                .setLinearHeadingInterpolation(pickupPose.getHeading(), shootPose.getHeading())
                .build();

        Park = new Path(new BezierLine(shootPose, park));
        Park.setLinearHeadingInterpolation(shootPose.getHeading(), park.getHeading());
    }

    public void autonomousPathUpdate(){
        switch (pathState){
            case 0:
                if (!follower.isBusy()){

                }

        }
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

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {}

    @Override
    public void start() {
    }

    @Override
    public void init_loop() {}
}
