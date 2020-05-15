package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "DragsterTest")
public class DragsterTest extends LinearOpMode {
    private GyroDragster bot;

    @Override
    public void runOpMode() throws InterruptedException {
        bot = new GyroDragster(telemetry);
        bot.initialize(hardwareMap);

        waitForStart();

        double init_yaw = bot.getYaw(AngleUnit.DEGREES);

        while(opModeIsActive()){
            bot.yawCorrectedForwardDrive(1.0, init_yaw);
            telemetry.update();
        }
    }
}
