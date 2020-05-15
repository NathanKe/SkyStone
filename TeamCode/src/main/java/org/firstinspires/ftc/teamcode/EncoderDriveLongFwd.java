package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "EncoderDrive-LongFwd")
public class EncoderDriveLongFwd extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumPlatformWithGyro bot = new MecanumPlatformWithGyro(telemetry);
        bot.initialize(hardwareMap);

        double avgTicks;

        waitForStart();
        sleep(500);
        bot.mecanumPlatform.resetDriveEncoders();
        double initialYaw = bot.getYaw(AngleUnit.DEGREES);
        do{
            avgTicks = bot.yawCorrectedDirectionDrive("Forward", 0.5, initialYaw);

            telemetry.update();
        }while(opModeIsActive() && avgTicks < 8000);

        bot.mecanumPlatform.stopAll();

    }
}
