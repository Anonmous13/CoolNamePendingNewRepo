package org.firstinspires.ftc.teamcode.opmode.tele;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.List;

@TeleOp(name = "Full TeleOp PID v10", group = "TeleOp")
public class FullTeleOpPIDV10 extends LinearOpMode {

    // Drive motors
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Shooter motors
    private DcMotor s1, s2;

    // Intake motor
    private DcMotor fi;

    // Servos
    private Servo ha;
    private CRServo tl, tr;
    private Servo armLeft, armBack, armRight;

    // Color Sensors
    private ColorSensor leftCs, backCs, rightCs;

    // Pinpoint
    private GoBildaPinpointDriver pinpoint;

    // Limelight
    private Limelight3A limelight;

    // Hood
    private double hoodPos = 0.4;
    private final double HOOD_MIN = 0.3;
    private final double HOOD_MAX = 0.9;
    private final double HOOD_STEP = 0.1;
    private final double HOOD_OPTIMAL = 0.4; // Best shooting angle

    // Shooter timing and auto-ramp
    long shooterStartTime = 0;
    boolean shooterActive = false;
    boolean shooterSpunUp = false;
    boolean autoRamping = false;
    long lastDetectionTime = 0;
    boolean wasDetecting = false;
    final long SPINUP_TIME_MS = 1450;
    final long BOOST_TIME_MS = 450;
    final long KEEP_ALIVE_TIME_MS = 5000; // Keep shooter on for 5 seconds after ball leaves
    final int COLOR_DETECT_THRESHOLD = 100; // Adjust based on your sensors

    // Arm sequence state
    private enum ArmState {
        IDLE,
        LIFTING_BACK,
        WAIT_BACK_DOWN,
        LIFTING_LEFT,
        WAIT_LEFT_DOWN,
        LIFTING_RIGHT,
        WAIT_RIGHT_DOWN
    }
    private ArmState armState = ArmState.IDLE;
    private long armMoveStartTime = 0;
    private final long ARM_MOVE_DELAY = 400; // Time to wait for arm movement (ms)

    boolean lastA = false, lastB = false;

    // AprilTag tracking
    private static final int TARGET_TAG_ID = 20;
    private static final double TRACK_DEADZONE = 1.5;
    private static final double TRACK_MAX_SPEED = 0.3;
    private static final double TRACK_MIN_SPEED = 0.05;
    private static final double TRACK_KP = 0.015;
    private static final double CLOSE_DISTANCE_INCHES = 60.0; // 5 feet = 60 inches
    private int framesOnTarget = 0;
    private static final int FRAMES_TO_LOCK = 3;

    @Override
    public void runOpMode() {

        // ---------------- HARDWARE MAP ----------------
        frontLeft  = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft   = hardwareMap.get(DcMotor.class, "bl");
        backRight  = hardwareMap.get(DcMotor.class, "br");

        s1 = hardwareMap.get(DcMotor.class, "s1");
        s2 = hardwareMap.get(DcMotor.class, "s2");
        fi = hardwareMap.get(DcMotor.class, "fi");

        ha = hardwareMap.get(Servo.class, "ha");
        tl = hardwareMap.get(CRServo.class, "tl");
        tr = hardwareMap.get(CRServo.class, "tr");

        armLeft  = hardwareMap.get(Servo.class, "al");
        armBack  = hardwareMap.get(Servo.class, "ab");
        armRight = hardwareMap.get(Servo.class, "ar");

        leftCs = hardwareMap.get(ColorSensor.class, "leftCs");
        backCs = hardwareMap.get(ColorSensor.class, "backCs");
        rightCs = hardwareMap.get(ColorSensor.class, "rightCs");

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        configurePinpoint();

        for (Limelight3A ll : hardwareMap.getAll(Limelight3A.class)) {
            limelight = ll;
            break;
        }

        if (limelight != null) {
            limelight.pipelineSwitch(1);
            limelight.start();
        }

        // ---------------- DIRECTIONS ----------------
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        s1.setDirection(DcMotor.Direction.REVERSE);
        s2.setDirection(DcMotor.Direction.REVERSE);
        fi.setDirection(DcMotor.Direction.REVERSE);

        // ---------------- BRAKE ----------------
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        s1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        s2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ---------------- INIT POS ----------------
        ha.setPosition(HOOD_OPTIMAL);
        armLeft.setPosition(1);
        armBack.setPosition(0.95);
        armRight.setPosition(1);

        waitForStart();

        while (opModeIsActive()) {

            // ---------------- DRIVE (NO PID) ----------------
            double y  = -gamepad1.left_stick_y;
            double x  = -gamepad1.left_stick_x;
            double rx =  gamepad1.right_stick_x;

            double fl = y + x + rx;
            double fr = y - x - rx;
            double bl = y - x + rx;
            double br = y + x - rx;

            double max = Math.max(
                    Math.abs(fl),
                    Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))
            );

            if (max > 1.0) {
                fl /= max;
                fr /= max;
                bl /= max;
                br /= max;
            }

            frontLeft.setPower(fl);
            frontRight.setPower(fr);
            backLeft.setPower(bl);
            backRight.setPower(br);

            // ---------------- HOOD MANUAL CONTROL ----------------
            // Manual control only - no auto-adjustment
            if (gamepad1.a && !lastA) {
                hoodPos += HOOD_STEP;
            } else if (gamepad1.b && !lastB) {
                hoodPos -= HOOD_STEP;
            }
            // Clamp hood position
            hoodPos = Math.max(HOOD_MIN, Math.min(HOOD_MAX, hoodPos));
            ha.setPosition(hoodPos);
            lastA = gamepad1.a;
            lastB = gamepad1.b;

            // ---------------- TURRET ----------------
            boolean manualTurret = gamepad1.dpad_left || gamepad1.dpad_right;

            if (manualTurret) {
                framesOnTarget = 0;
                double p = 0.5;
                if (gamepad1.dpad_left) {
                    tl.setPower(-p);
                    tr.setPower(-p);
                } else if (gamepad1.dpad_right) {
                    tl.setPower(p);
                    tr.setPower(p);
                } else {
                    tl.setPower(0);
                    tr.setPower(0);
                }
            }
            else {
                autoTrackAprilTag();
            }

            // ---------------- INTAKE ----------------
            if (gamepad2.x) fi.setPower(1);
            else if (gamepad2.y) fi.setPower(-1);
            else fi.setPower(0);

            // ---------------- ARMS WITH AUTO SEQUENCE ----------------
            // Manual arm control with individual buttons
            if (gamepad2.dpad_left && armState == ArmState.IDLE) {
                armBack.setPosition(0.5);
            }
            if (gamepad2.dpad_right && armState == ArmState.IDLE) {
                armLeft.setPosition(0.5);
            }

            // Reset all arms to down position
            if (gamepad2.dpad_down) {
                armLeft.setPosition(1);
                armBack.setPosition(0.95);
                armRight.setPosition(1);
                armState = ArmState.IDLE;
            }

            // Auto sequence - lift only arms where color sensors detect objects
            if (gamepad2.dpad_up && armState == ArmState.IDLE) {
                // Start the sequence - check which arms need lifting
                armState = determineNextArmToLift();
                if (armState != ArmState.IDLE) {
                    armMoveStartTime = System.currentTimeMillis();
                }
            }

            // Process arm sequence state machine
            processArmSequence();

            // ---------------- COLOR SENSOR DETECTION ----------------
            boolean objectDetected = detectObject();

            // Track when object leaves sensor - keep shooter on for 5 seconds after
            if (objectDetected) {
                lastDetectionTime = System.currentTimeMillis();
                wasDetecting = true;
            }

            // Check if we're still within the keep-alive window
            long timeSinceDetection = System.currentTimeMillis() - lastDetectionTime;
            boolean keepShooterOn = wasDetecting && (timeSinceDetection < KEEP_ALIVE_TIME_MS);

            // ---------------- SHOOTER WITH AUTO-RAMP ----------------
            double targetPower = 0;
            boolean manualShoot = gamepad2.a || gamepad2.b;

            // Manual shooting (gamepad overrides)
            if (manualShoot) {
                if (gamepad2.a) targetPower = 0.55;
                else if (gamepad2.b) targetPower = 0.45;
                else if (gamepad1.x) targetPower =1;
                autoRamping = false;
                wasDetecting = false; // Reset auto-ramp state when manual shooting
            }
            // Auto-ramp when object detected OR within keep-alive window
            // Always use 0.45 power for auto-ramp
            else if (objectDetected || keepShooterOn) {
                targetPower = 0.45;
                autoRamping = true;
            } else {
                autoRamping = false;
                wasDetecting = false; // Reset after keep-alive expires
            }

            double power = 0;
            if (targetPower > 0) {
                if (!shooterActive) {
                    shooterStartTime = System.currentTimeMillis();
                    shooterActive = true;
                    shooterSpunUp = false;
                }

                long elapsed = System.currentTimeMillis() - shooterStartTime;

                power = (elapsed < BOOST_TIME_MS) ? 1.0 : targetPower;

                if (!shooterSpunUp && elapsed >= SPINUP_TIME_MS) {
                    if (!autoRamping) { // Only rumble on manual shoot
                        gamepad2.rumble(1, 1, 800);
                    }
                    shooterSpunUp = true;
                }
            } else {
                shooterActive = false;
                shooterSpunUp = false;
            }

            s1.setPower(power);
            s2.setPower(power);

            // ---------------- TELEMETRY ----------------
            pinpoint.update();
            Pose2D pos = pinpoint.getPosition();

            telemetry.addData("Hood", hoodPos);
            telemetry.addData("Shooter Power", power);
            telemetry.addData("Target Power", targetPower);
            telemetry.addData("Auto-Ramping", autoRamping);
            telemetry.addData("Keep Shooter On", keepShooterOn);
            telemetry.addData("Time Since Detection", timeSinceDetection + "ms");
            telemetry.addData("Left Sensor", leftCs.alpha());
            telemetry.addData("Back Sensor", backCs.alpha());
            telemetry.addData("Right Sensor", rightCs.alpha());
            telemetry.addData("Object Detected", objectDetected);
            telemetry.addData("Arm State", armState);
            telemetry.addData("X", pos.getX(DistanceUnit.INCH));
            telemetry.addData("Y", pos.getY(DistanceUnit.INCH));
            telemetry.addData("Heading", Math.toDegrees(pos.getHeading(AngleUnit.RADIANS)));
            telemetry.update();
        }
    }

    // ---------------- COLOR SENSOR DETECTION ----------------
    private boolean detectObject() {
        // Check if any color sensor detects an object
        // Using alpha() for light intensity detection
        return leftCs.alpha() > COLOR_DETECT_THRESHOLD ||
                backCs.alpha() > COLOR_DETECT_THRESHOLD ||
                rightCs.alpha() > COLOR_DETECT_THRESHOLD;
    }

    // ---------------- DISTANCE-BASED POWER ----------------
    private double getDistanceToTarget() {
        if (limelight == null) return -1;

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
            for (LLResultTypes.FiducialResult tag : tags) {
                if (tag.getFiducialId() == TARGET_TAG_ID) {
                    // Use target area for distance estimation
                    if (tag.getTargetArea() > 0) {
                        // Larger area = closer distance
                        // This formula needs calibration for your specific setup
                        // Example: area of 5.0 at 60 inches, area of 2.0 at 120 inches
                        double distance = 300.0 / Math.sqrt(tag.getTargetArea());
                        return distance;
                    }
                }
            }
        }
        return -1; // No target found
    }

    private double getDistanceBasedPower() {
        double distance = getDistanceToTarget();

        // If we can't get distance from Limelight, don't auto-ramp
        if (distance < 0) {
            return 0; // Return 0 to indicate no auto power
        }

        // If within 5 feet (60 inches), use lower power (0.4)
        if (distance <= CLOSE_DISTANCE_INCHES) {
            return 0.45;
        } else {
            // Farther than 5 feet, use higher power (0.45)
            return 0.55;
        }
    }

    // ---------------- ARM SEQUENCE CONTROL ----------------
    private ArmState determineNextArmToLift() {
        // Check color sensors and determine which arm to lift
        // Match sensor to correct arm: leftCs -> armLeft, backCs -> armBack, rightCs -> armRight
        boolean backDetected = backCs.alpha() > COLOR_DETECT_THRESHOLD;
        boolean leftDetected = leftCs.alpha() > COLOR_DETECT_THRESHOLD;
        boolean rightDetected = rightCs.alpha() > COLOR_DETECT_THRESHOLD;
        if (gamepad1.dpad_up) {
            armBack.setPosition(0.5);

        }
        if (gamepad1.dpad_down) {
            armBack.setPosition(0.9);
        }

        // Start with back if it has object
        if (backDetected) {
            armBack.setPosition(0.5);
            return ArmState.LIFTING_BACK;
        } else if (rightDetected) {
            armRight.setPosition(0.5);
            return ArmState.LIFTING_RIGHT;
        } else if (leftDetected) {
            armLeft.setPosition(0.5);
            return ArmState.LIFTING_LEFT;
        }
        return ArmState.IDLE; // No objects detected
    }

    private void processArmSequence() {
        long elapsed = System.currentTimeMillis() - armMoveStartTime;

        switch (armState) {
            case LIFTING_BACK:
                if (elapsed > ARM_MOVE_DELAY) {
                    // Lower back arm
                    armBack.setPosition(0.95);
                    armState = ArmState.WAIT_BACK_DOWN;
                    armMoveStartTime = System.currentTimeMillis();
                }
                break;

            case WAIT_BACK_DOWN:
                if (elapsed > ARM_MOVE_DELAY) {
                    // Move to next arm only if it has an object
                    if (rightCs.alpha() > COLOR_DETECT_THRESHOLD) {
                        armRight.setPosition(0.5);
                        armState = ArmState.LIFTING_RIGHT;
                        armMoveStartTime = System.currentTimeMillis();
                    } else if (leftCs.alpha() > COLOR_DETECT_THRESHOLD) {
                        armLeft.setPosition(0.5);
                        armState = ArmState.LIFTING_LEFT;
                        armMoveStartTime = System.currentTimeMillis();
                    } else {
                        armState = ArmState.IDLE; // Done - no more objects
                    }
                }
                break;

            case LIFTING_RIGHT:
                if (elapsed > ARM_MOVE_DELAY) {
                    // Lower right arm
                    armRight.setPosition(1);
                    armState = ArmState.WAIT_RIGHT_DOWN;
                    armMoveStartTime = System.currentTimeMillis();
                }
                break;

            case WAIT_RIGHT_DOWN:
                if (elapsed > ARM_MOVE_DELAY) {
                    // Move to left arm only if it has an object
                    if (leftCs.alpha() > COLOR_DETECT_THRESHOLD) {
                        armLeft.setPosition(0.5);
                        armState = ArmState.LIFTING_LEFT;
                        armMoveStartTime = System.currentTimeMillis();
                    } else {
                        armState = ArmState.IDLE; // Done - no more objects
                    }
                }
                break;

            case LIFTING_LEFT:
                if (elapsed > ARM_MOVE_DELAY) {
                    // Lower left arm
                    armLeft.setPosition(1);
                    armState = ArmState.WAIT_LEFT_DOWN;
                    armMoveStartTime = System.currentTimeMillis();
                }
                break;

            case WAIT_LEFT_DOWN:
                if (elapsed > ARM_MOVE_DELAY) {
                    // Sequence complete - all arms done
                    armState = ArmState.IDLE;
                }
                break;

            case IDLE:
            default:
                break;
        }
    }

    // ---------------- HOOD ANGLE CALCULATION ----------------
    private double calculateHoodAngle(double distance) {
        // Maps distance to hood angle, keeping it close to optimal 0.4
        // Small adjustments for different distances

        // Define distance ranges (in inches)
        final double CLOSE_DISTANCE = 50.0;   // ~4 feet
        final double MEDIUM_DISTANCE = 80.0;  // ~6.5 feet
        final double FAR_DISTANCE = 120.0;    // ~10 feet

        if (distance < CLOSE_DISTANCE) {
            // Very close - slightly lower angle
            return 0.35;
        } else if (distance < MEDIUM_DISTANCE) {
            // Medium range - optimal angle
            return 0.4;
        } else if (distance < FAR_DISTANCE) {
            // Medium-far range - slightly higher
            return 0.45;
        } else {
            // Far range - higher angle
            return 0.5;
        }
    }

    // ---------------- APRILTAG TRACK ----------------
    private void autoTrackAprilTag() {
        if (limelight == null) {
            tl.setPower(0);
            tr.setPower(0);
            return;
        }

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

            for (LLResultTypes.FiducialResult tag : tags) {
                if (tag.getFiducialId() == TARGET_TAG_ID) {
                    double tx = tag.getTargetXDegrees();

                    if (Math.abs(tx) < TRACK_DEADZONE) {
                        tl.setPower(0);
                        tr.setPower(0);
                        return;
                    }

                    double speed = tx * TRACK_KP;
                    speed = Math.max(TRACK_MIN_SPEED, Math.min(TRACK_MAX_SPEED, Math.abs(speed)))
                            * Math.signum(speed);

                    tl.setPower(speed);
                    tr.setPower(speed);
                    return;
                }
            }
        }

        tl.setPower(0);
        tr.setPower(0);
    }

    private void configurePinpoint() {
        pinpoint.setOffsets(0, 0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
    }
}