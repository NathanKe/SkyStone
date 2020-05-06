package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "BangBang Front Left")
public class BangBang extends LinearOpMode {

    private static final int forwardTicks = 4000;

    @Override
    public void runOpMode() {
        MecanumPlatform bot = new MecanumPlatform(telemetry);
        bot.initialize(hardwareMap, false);

        bot.motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while(opModeIsActive()){
            int motorFLcurrentPos = bot.motorFL.getCurrentPosition();
            if(motorFLcurrentPos <= forwardTicks) {
                bot.motorFL.setPower(1.0);
            } else {
                bot.motorFL.setPower(-1.0);
            }
            telemetry.addData("Pos", motorFLcurrentPos);
            telemetry.addData("Pow", bot.motorFL.getPower());
            telemetry.update();
        }
    }
}
