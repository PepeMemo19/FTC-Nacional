package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.Bot;

@com.acmerobotics.dashboard.config.Config
public class Claw extends SubsystemBase {

    private final Bot bot;

    public final Servo clawPivot;

    public final  double treshold = 2;
    private double currentPosition = ServoPositions.placing;

    @com.acmerobotics.dashboard.config.Config
    public static class ServoPositions {
        public static double
                placing = 0.6,
                safe = 0.3,
                intaking = 0.15,
                safe2= 0.15;
    }


    public Claw(Bot bot) {
        this.bot = bot;
        clawPivot = bot.hMap.get(Servo.class, "claw");
    }

    @Override
    public void periodic() {
        clawPivot.setPosition(currentPosition);
    }

    public void setPosition(double position){
        currentPosition = position;
    }



}