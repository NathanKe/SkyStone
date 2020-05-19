package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Tele_Dragster_Basic")
public class Tele_Dragster_Basic extends OpMode {
    private DragsterPlatform bot;

    @Override
    public void init() {
        bot = new DragsterPlatform(telemetry);
        bot.initialize(hardwareMap);
    }

    @Override
    public void loop() {
        double fwd_bck = -1 * gamepad1.left_stick_y;
        double lft_rgt = gamepad1.right_stick_x;


        double power_left = fwd_bck + lft_rgt;
        double power_right = -1 * fwd_bck + lft_rgt;

        bot.setMotorPowers(power_left, power_right);

        telemetry.update();
    }

    @Override
    public void stop() {
        bot.stopMotors();
    }
}
