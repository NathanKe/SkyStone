package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.text.DecimalFormat;
import java.util.Arrays;
import java.util.Collections;

class MecanumPlatform {

    private Telemetry telemetry;

    public DcMotor motorFL;
    private DcMotor motorFR;
    private DcMotor motorBL;
    private DcMotor motorBR;

    private BNO055IMU imu;

    private DecimalFormat twoDecimalPlaces = new DecimalFormat("#.##");

    MecanumPlatform(Telemetry in_telemetry) {
        this.telemetry = in_telemetry;
    }

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

        motorFL = in_hwMap.get(DcMotor.class, "motorFL");
        motorFR = in_hwMap.get(DcMotor.class, "motorFR");
        motorBL = in_hwMap.get(DcMotor.class, "motorBL");
        motorBR = in_hwMap.get(DcMotor.class, "motorBR");
        stopAll();
    }

    public double getYaw(AngleUnit degreeOrRadian){
        return -1 * imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, degreeOrRadian).thirdAngle;
    }

    void baseDriveTrain(double left_x, double left_y, double right_x, double scale) {

        double raw_fl = left_y + left_x + right_x;
        double raw_fr = -1 * left_y + left_x + right_x;
        double raw_bl = left_y + -1 * left_x + right_x;
        double raw_br = -1 * left_y + -1 * left_x + right_x;

        double max = Collections.max(Arrays.asList(raw_fl, raw_fr, raw_bl, raw_br, 1.0));

        double out_fl = raw_fl / (max * scale);
        double out_fr = raw_fr / (max * scale);
        double out_bl = raw_bl / (max * scale);
        double out_br = raw_br / (max * scale);

        setDrivePower(out_fl, out_fr, out_bl, out_br);

        String out_fl_st = twoDecimalPlaces.format(out_fl);
        String out_fr_st = twoDecimalPlaces.format(out_fr);
        String out_bl_st = twoDecimalPlaces.format(out_bl);
        String out_br_st = twoDecimalPlaces.format(out_br);
        String motor_str = out_fl_st + ", " + out_fr_st + ", " + out_bl_st + ", " + out_br_st;


        telemetry.addData("Motors: ", motor_str);
        telemetry.update();

    }

    void fieldDriveTrain(double left_x, double left_y, double right_x, double scale) {
        double heading = getYaw(AngleUnit.RADIANS);

        double adj_left_x = left_x * Math.cos(heading) - left_y * Math.sin(heading);
        double adj_left_y = left_x * Math.sin(heading) + left_y * Math.cos(heading);

        telemetry.addData("Heading: ", Math.round(heading * 180 / Math.PI));
        telemetry.addData("X", twoDecimalPlaces.format(left_x));
        telemetry.addData("X adj", twoDecimalPlaces.format(adj_left_x));
        telemetry.addData("Y", twoDecimalPlaces.format(left_y));
        telemetry.addData("Y adj", twoDecimalPlaces.format(left_y));

        baseDriveTrain(adj_left_x, adj_left_y, right_x, scale);
    }

    private void stopDriveMotors() {
        motorFL.setPower(0.0);
        motorFR.setPower(0.0);
        motorBL.setPower(0.0);
        motorBR.setPower(0.0);
    }

    void stopAll() {
        stopDriveMotors();
    }

    void resetDriveEncoders() {
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void setDriveTarget(int ticks) {
        motorFL.setTargetPosition(ticks);
        motorFR.setTargetPosition(ticks);
        motorBL.setTargetPosition(ticks);
        motorBR.setTargetPosition(ticks);

        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    void setDrivePower(double powerFL, double powerFR, double powerBL, double powerBR) {
        double max = Math.max(Math.max(Math.abs(powerFL), Math.abs(powerFR)), Math.max(Math.abs(powerBL), Math.abs(powerBR)));

        // Too small, cut to zero
        if(max <= 0.03){
            motorFL.setPower(0.0);
            motorFR.setPower(0.0);
            motorBL.setPower(0.0);
            motorBR.setPower(0.0);
        }
        // Normal power, send it!
        else{
            motorFL.setPower(powerFL);
            motorFR.setPower(powerFR);
            motorBL.setPower(powerBL);
            motorBR.setPower(powerBR);
        }
    }

    void driveEncoderTelemetryReadout() {
        telemetry.addData("FL", motorFL.getCurrentPosition());
        telemetry.addData("FR", motorFR.getCurrentPosition());
        telemetry.addData("BL", motorBL.getCurrentPosition());
        telemetry.addData("BR", motorBR.getCurrentPosition());
        telemetry.update();
    }

    boolean driveMotorsBusy() {
        return motorFL.isBusy() && motorFR.isBusy() && motorBL.isBusy() && motorBR.isBusy();
    }
}
