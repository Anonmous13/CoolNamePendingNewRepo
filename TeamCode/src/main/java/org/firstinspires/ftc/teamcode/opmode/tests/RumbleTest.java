package org.firstinspires.ftc.teamcode.opmode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Rumble Test")
public class RumbleTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initialized");
        waitForStart();
        while(opModeIsActive()) {
            gamepad1.rumbleBlips(3);
            sleep(1000);
            gamepad1.rumble(1000);
            gamepad1.stopRumble();
        }
    }
}
