package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;

@TeleOp(name = "Gyro")
public class GyroStuff extends OpMode {
    private BNO055IMU imu;
    private DcMotor axis;

    private double pid_p = 0.063;
    private double pid_i = 0.00;
    private double pid_d = 0.00;

    private double error;
    private double errorPrev;
    private double errorSum;
    private double errorDif;
    private double loopCount;

    private double calcPower;

    private void initialize_gyro(HardwareMap in_hwMap) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = in_hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    void initialize(HardwareMap in_hwMap, boolean useGyro) {
        if (useGyro) {
            initialize_gyro(in_hwMap);
        }
        axis = in_hwMap.get(DcMotor.class, "axis");
        axis.setDirection(DcMotorSimple.Direction.REVERSE);
        axis.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        errorPrev = 0;
        errorSum = 0;
        errorDif = 0;
        loopCount = 0;
    }

    @Override
    public void init() {
        initialize(hardwareMap, true);
    }

    @Override
    public void loop() {
        Acceleration gravity = imu.getGravity();

        double xAccel = gravity.xAccel;
        double yAccel = gravity.yAccel;
        double zAccel = gravity.zAccel;

        axis.setPower(gamepad1.left_stick_x);

        if(xAccel <= 0){
            error = 10 * Math.signum(yAccel);
        } else {
            error = yAccel;
            loopCount++;
        }

        //loopCount++;
        errorSum += error;
        errorDif = Math.abs(error) - Math.abs(errorPrev);
        errorPrev = error;

        calcPower = pid_p * error + pid_i * errorSum / loopCount + pid_d + errorDif;

        axis.setPower(calcPower);


        telemetry.addData("Pow", calcPower);
        telemetry.addData("Err", error);
        telemetry.addData("ESum", errorSum);
        telemetry.addData("EDif", errorDif);
        telemetry.addData("XA", xAccel);
        telemetry.addData("YA", yAccel);
        telemetry.addData("ZA", zAccel);
        telemetry.addData("Total", Math.sqrt(xAccel*xAccel + yAccel*yAccel + zAccel*zAccel));
        telemetry.update();

    }
}
