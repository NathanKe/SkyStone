package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class GyroDragster {
    private Telemetry telemetry;

    private BNO055IMU imu;
    private DcMotor leftMotor;
    private DcMotor rightMotor;

    private final double YAW_CORRECT_PROPORTION_CONSTANT = 0.05;

    GyroDragster(Telemetry in_telemetry){this.telemetry = in_telemetry;}

    void initialize(HardwareMap in_hwMap){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = in_hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        leftMotor = in_hwMap.get(DcMotor.class, "motorLeft");
        rightMotor = in_hwMap.get(DcMotor.class, "motorRight");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    double getYaw(AngleUnit degreeOrRadian) {
        return -1 * imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, degreeOrRadian).thirdAngle;
    }

    void yawCorrectedForwardDrive(double base_power, double initialYaw_degrees){
        double currentYaw = getYaw(AngleUnit.DEGREES);
        double yawCorrectFactor = YAW_CORRECT_PROPORTION_CONSTANT * (initialYaw_degrees - currentYaw);

        double powerLeft = 1.0 * base_power + yawCorrectFactor;
        double powerRight = -1.0 * base_power + yawCorrectFactor;

        leftMotor.setPower(powerLeft);
        rightMotor.setPower(powerRight);

        telemetry.addData("Left", powerLeft);
        telemetry.addData("Right", powerRight);
        telemetry.addData("Yaw", currentYaw);
        telemetry.addData("YawDiff", initialYaw_degrees - currentYaw);
    }

}
