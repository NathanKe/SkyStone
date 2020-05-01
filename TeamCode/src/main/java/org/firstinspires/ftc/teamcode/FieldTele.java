package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "FieldTele")
public class FieldTele extends OpMode {

    private MecanumPlatform bot;

    @Override
    public void init() {
        bot = new MecanumPlatform(telemetry);
        bot.initialize(hardwareMap, true);
    }

    @Override
    public void loop() {
        double left_x = gamepad1.left_stick_x;
        double left_y = -1 * gamepad1.left_stick_y; // handle invert y
        double right_x = gamepad1.right_stick_x;
        bot.fieldDriveTrain(left_x, left_y, right_x, 1.0);
    }

    @Override
    public void stop() {
        bot.stopAll();
    }
}