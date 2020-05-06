package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "PID Turn")
public class PidTurn extends LinearOpMode {

    private static final double turnAngle = -90.0;

    private static final double PID_P = 0.012;
    private static final double PID_I = 0.0005;
    private static final double PID_D = 0.01;

    private double error;
    private double errorPrev;
    private double errorSum;
    private double errorDif;
    private double calculatedPower;
    private int loopCount;

    @Override
    public void runOpMode(){
        MecanumPlatform bot = new MecanumPlatform(telemetry);
        bot.initialize(hardwareMap, true);

        errorPrev = 0;
        errorSum = 0;
        errorDif = 0;
        loopCount = 0;

        waitForStart();

        while(opModeIsActive()){
            double heading = bot.getYaw(AngleUnit.DEGREES);
            error = turnAngle - heading;

            loopCount ++;
            errorSum += error;
            errorDif = error - errorPrev;
            errorPrev = error;

            calculatedPower = PID_P * error + PID_I * errorSum / loopCount + PID_D * errorDif;
            bot.setDrivePower(calculatedPower, calculatedPower, calculatedPower, calculatedPower);

            telemetry.addData("Loop", loopCount);
            telemetry.addData("Hea", heading);
            telemetry.addData("Err", error);
            telemetry.addData("ErrSum", errorSum);
            telemetry.addData("ErrInt", errorSum/loopCount);
            telemetry.addData("ErrDif", errorDif);
            telemetry.addData("Calc", calculatedPower);
            telemetry.addData("Pow", bot.motorFL.getPower());
            telemetry.update();
        }
    }
}
