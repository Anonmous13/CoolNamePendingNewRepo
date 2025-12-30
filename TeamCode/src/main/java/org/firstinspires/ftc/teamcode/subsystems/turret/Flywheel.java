package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Flywheel {
    private DcMotor s1, s2;
    private CRServo t1, t2;
    private Servo f1, f2, f3, angleServo;
    private ElapsedTime stateTimer;

    public enum TurretState {
        IDLE,
        SHOOT,
    }
    private TurretState turretState;

    private double TurretPower = 0.55;
    private double angleServoPos = 0.4;
    private double turretServoPower = 1.0;
}