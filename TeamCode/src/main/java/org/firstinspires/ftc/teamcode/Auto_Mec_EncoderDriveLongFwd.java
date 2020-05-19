package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "Auto_Mec_EncoderDrive-LongFwd")
public class Auto_Mec_EncoderDriveLongFwd extends LinearOpMode {
    @Override
    public void runOpMode() {
        GyroMecanum bot = new GyroMecanum(telemetry);
        bot.initialize(hardwareMap);

        double avgTicks;

        waitForStart();
        sleep(500);
        bot.mecanumPlatform.resetDriveEncoders();
        double initialYaw = bot.gyro.getYaw(AngleUnit.DEGREES);
        do {
            avgTicks = bot.yawCorrectedDirectionDrive("Forward", 0.5, initialYaw);

            telemetry.update();
        } while (opModeIsActive() && avgTicks < 8000);

        bot.mecanumPlatform.stopAll();

    }
}
