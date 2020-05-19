package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

class GyroDragster {
    private Telemetry telemetry;

    DragsterPlatform dragsterPlatform;
    Gyro gyro;

    private DcMotor leftMotor;
    private DcMotor rightMotor;

    private final double YAW_CORRECT_PROPORTION_CONSTANT = 0.02;

    GyroDragster(Telemetry in_telemetry) {
        this.telemetry = in_telemetry;
    }

    void initialize(HardwareMap in_hwMap) {
        dragsterPlatform = new DragsterPlatform(telemetry);
        dragsterPlatform.initialize(in_hwMap);

        gyro = new Gyro(telemetry);
        gyro.initialize(in_hwMap);
    }

    void yawCorrectedForwardDrive(double base_power, double initialYaw_degrees) {
        double currentYaw = gyro.getYaw(AngleUnit.DEGREES);
        double yawCorrectFactor = YAW_CORRECT_PROPORTION_CONSTANT * (initialYaw_degrees - currentYaw);

        double powerLeft = 1.0 * base_power + yawCorrectFactor;
        double powerRight = -1.0 * base_power + yawCorrectFactor;

        dragsterPlatform.setMotorPowers(powerLeft, powerRight);

        telemetry.addData("Yaw", currentYaw);
        telemetry.addData("YawDiff", initialYaw_degrees - currentYaw);
    }
}
