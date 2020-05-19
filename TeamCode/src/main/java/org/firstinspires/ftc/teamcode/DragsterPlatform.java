package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

class DragsterPlatform {
    private Telemetry telemetry;

    private DcMotor leftMotor;
    private DcMotor rightMotor;

    private final double YAW_CORRECT_PROPORTION_CONSTANT = 0.02;

    DragsterPlatform(Telemetry in_telemetry) {
        this.telemetry = in_telemetry;
    }

    void initialize(HardwareMap in_hwMap) {
        leftMotor = in_hwMap.get(DcMotor.class, "motorLeft");
        rightMotor = in_hwMap.get(DcMotor.class, "motorRight");
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    void setMotorPowers(double power_left, double power_right) {
        leftMotor.setPower(power_left);
        rightMotor.setPower(power_right);

        telemetry.addData("Left", power_left);
        telemetry.addData("Right", power_right);
    }

    void stopMotors() {
        setMotorPowers(0, 0);
    }

}
