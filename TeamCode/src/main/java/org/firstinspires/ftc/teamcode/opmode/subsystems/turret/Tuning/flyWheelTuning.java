package org.firstinspires.ftc.teamcode.opmode.subsystems.turret.Tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


//TODO: Tune the PIDF for the Fly Wheel so that the speed can be extremely fast so our shooter can shoot from far

@TeleOp(name = "Fly Wheel Tuner")
public class flyWheelTuning extends OpMode {
    public DcMotorEx flywheelMotor;
    public double highWheelVelocity = 1500;
    public double lowVelocity = 900;
    double currentTargetVelocity = highWheelVelocity;
    double F = 0;
    double P = 0;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};

    int stepIndex = 1;
    @Override
    public void init() {
        //stuff for the motor
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "turret_motor");
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //PIDF stuff
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("Init Complete, Press Play to begin tuning!");
    }

    @Override
    public void loop() {
        //get all gamepad commands
        //set target velocity
        //update telemetry

        if (gamepad1.yWasPressed()){
            if (currentTargetVelocity == highWheelVelocity) {
                currentTargetVelocity = lowVelocity;
            } else { currentTargetVelocity = highWheelVelocity; }
        }

        if (gamepad1.bWasPressed()){
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()){
            F -= stepSizes[stepIndex];
        }

        if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }

        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }

        if (gamepad1.dpadDownWasPressed()){
            P -= stepSizes[stepIndex];
        }

        //set new PIDF coefficients
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        //set velocity
        flywheelMotor.setVelocity(currentTargetVelocity);

        double curVelocity = flywheelMotor.getVelocity();
        double error = currentTargetVelocity - curVelocity;

        telemetry.addData("Current Target Velocity", currentTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", curVelocity);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addLine("---------------------------------");
        telemetry.addLine("To tune P, click the D-Pad Up to increase and D-Pad Down to decrease");
        telemetry.addData("Tuning P", "%.4f", P);
        telemetry.addLine("To tune F, click the D-Pad Right to increase and D-Pad Left to decrease");
        telemetry.addData("Tuning P", "%.4f", P);
        telemetry.addData("Tuning F", "%.4f", F);
        telemetry.addData("Step Sizes", "%.4f (B)", stepSizes[stepIndex]);
        telemetry.update();
    }
}
