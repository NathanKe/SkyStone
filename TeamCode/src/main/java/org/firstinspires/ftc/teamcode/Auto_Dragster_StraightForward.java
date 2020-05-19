package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "DragsterStraightForward")
public class Auto_Dragster_StraightForward extends LinearOpMode {
    private GyroDragster bot;

    @Override
    public void runOpMode() {
        bot = new GyroDragster(telemetry);
        bot.initialize(hardwareMap);

        waitForStart();

        double init_yaw = bot.gyro.getYaw(AngleUnit.DEGREES);

        while (opModeIsActive()) {
            bot.yawCorrectedForwardDrive(1.0, init_yaw);
            telemetry.update();
        }
    }
}
