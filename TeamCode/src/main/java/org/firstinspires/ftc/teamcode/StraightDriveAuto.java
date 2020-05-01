package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "StraightDriveAuto")
public class StraightDriveAuto extends LinearOpMode {

    private static final int forwardTicks = 4000;

    @Override
    public void runOpMode() {
        MecanumPlatform bot = new MecanumPlatform(telemetry);
        bot.initialize(hardwareMap, true);

        bot.resetDriveEncoders();
        bot.setDriveTarget(forwardTicks);
        bot.setDrivePower(.1, .1, .1, .1);

        while (opModeIsActive() && bot.driveMotorsBusy()) {
            bot.driveEncoderTelemetryReadout();
        }

        bot.stopAll();
    }
}
