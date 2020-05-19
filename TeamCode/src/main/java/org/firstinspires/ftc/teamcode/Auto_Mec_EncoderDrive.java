package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Auto_Mec_EncoderDrive")
public class Auto_Mec_EncoderDrive extends LinearOpMode {
    @Override
    public void runOpMode() {
        GyroMecanum bot = new GyroMecanum(telemetry);
        bot.initialize(hardwareMap);

        waitForStart();
        sleep(500);
        // bot.directionByTicks("Forward", 1000, 0.5);
        sleep(200);
        //  bot.directionByTicks("Left", 1000, 0.5);
        sleep(200);
        // bot.directionByTicks("Backward", 1000, 0.5);
        sleep(200);
        //bot.directionByTicks("Right", 1000, 0.5);
        bot.mecanumPlatform.stopAll();
    }
}
