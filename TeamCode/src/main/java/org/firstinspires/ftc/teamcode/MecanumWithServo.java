package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MecanumWithServo{

    private Telemetry telemetry;

    public MecanumPlatform mecanumPlatform;

    private CRServo servo;

    MecanumWithServo(Telemetry in_telemetry) {
        this.telemetry = in_telemetry;
        mecanumPlatform = new MecanumPlatform(this.telemetry);
    }

    void initialize(HardwareMap in_hwMap, boolean useGyro){
        mecanumPlatform.initialize(in_hwMap, useGyro);
        servo = in_hwMap.get(CRServo.class, "servo");
    }

    void turnServo(double servoPower){
        servo.setPower(servoPower);
        telemetry.addData("servoPower", servoPower);
    }
}
