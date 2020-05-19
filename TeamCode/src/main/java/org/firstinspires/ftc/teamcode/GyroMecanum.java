package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.text.DecimalFormat;

class GyroMecanum {

    private final double YAW_CORRECT_PROPORTION_CONSTANT = 0.7;

    MecanumPlatform mecanumPlatform;
    Gyro gyro;

    private Telemetry telemetry;

    private DecimalFormat twoDecimalPlaces = new DecimalFormat("#.##");

    GyroMecanum(Telemetry in_telemetry) {
        this.telemetry = in_telemetry;
    }


    void initialize(HardwareMap in_hwMap) {
        mecanumPlatform = new MecanumPlatform(telemetry);
        mecanumPlatform.initialize(in_hwMap);
        gyro = new Gyro(telemetry);
        gyro.initialize(in_hwMap);
    }

    void fieldDriveTrain(double left_x, double left_y, double right_x, double scale) {
        double heading = gyro.getYaw(AngleUnit.RADIANS);

        double adj_left_x = left_x * Math.cos(heading) - left_y * Math.sin(heading);
        double adj_left_y = left_x * Math.sin(heading) + left_y * Math.cos(heading);

        telemetry.addData("Heading: ", Math.round(heading * 180 / Math.PI));
        telemetry.addData("X", twoDecimalPlaces.format(left_x));
        telemetry.addData("X adj", twoDecimalPlaces.format(adj_left_x));
        telemetry.addData("Y", twoDecimalPlaces.format(left_y));
        telemetry.addData("Y adj", twoDecimalPlaces.format(left_y));

        mecanumPlatform.baseDriveTrain(adj_left_x, adj_left_y, right_x, scale);
    }

    double yawCorrectedDirectionDrive(String direction, double power, double initialYaw_degrees) {
        int powerSignFL;
        int powerSignFR;
        int powerSignBL;
        int powerSignBR;

        int[] ticksArr;

        double ticksAvg;

        double currentYaw;
        double yawCorrectFactor;

        switch (direction) {
            case "Forward":
                powerSignFL = 1;
                powerSignFR = -1;
                powerSignBL = 1;
                powerSignBR = -1;
                break;
            case "Backward":
                powerSignFL = -1;
                powerSignFR = 1;
                powerSignBL = -1;
                powerSignBR = 1;
                break;
            case "Left":
                powerSignFL = -1;
                powerSignFR = -1;
                powerSignBL = 1;
                powerSignBR = 1;
                break;
            case "Right":
                powerSignFL = 1;
                powerSignFR = 1;
                powerSignBL = -1;
                powerSignBR = -1;
                break;
            default:
                powerSignFL = 0;
                powerSignFR = 0;
                powerSignBL = 0;
                powerSignBR = 0;
        }

        currentYaw = gyro.getYaw(AngleUnit.DEGREES);
        yawCorrectFactor = YAW_CORRECT_PROPORTION_CONSTANT * (initialYaw_degrees - currentYaw);
        mecanumPlatform.setDrivePower(power * powerSignFL + yawCorrectFactor,
                power * powerSignFR + yawCorrectFactor,
                power * powerSignBL + yawCorrectFactor,
                power * powerSignBR + yawCorrectFactor);

        ticksArr = mecanumPlatform.getDriveCurrentPositionArray();

        ticksAvg = (powerSignFL * ticksArr[0] + powerSignFR * ticksArr[1] + powerSignBL * ticksArr[2] + powerSignBR * ticksArr[3]) / 4.0;

        telemetry.addData("Yaw", currentYaw);
        telemetry.addData("CF", yawCorrectFactor);
        telemetry.addData("TickAvg", ticksAvg);

        return ticksAvg;
    }
}
