package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;

public class Arm {
    private Servo arm;
    private HardwareMap hardwareMap;

    public Arm(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;

        arm = hardwareMap.get(Servo.class, "arm");
    }


}
