package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.Bot;

@com.acmerobotics.dashboard.config.Config
public class Claw extends SubsystemBase {

    private final Bot bot;

    public final Servo clawPivot;
    public final Servo claw;

    public final  double treshold = 2;
    private double currentPivotPosition = ServoPositions.placing;
    private double currentClawState = ClawStates.close;

    @com.acmerobotics.dashboard.config.Config
    public static class ServoPositions {
        public static double placing = 0.6, safe = 0.3, intaking = 0.15, safe2= 0.15;
    }

    @Config
    public static class ClawStates {
        public static double open = 0.1, close = 0.9;
    }


    public Claw(Bot bot) {
        this.bot = bot;
        clawPivot = bot.hMap.get(Servo.class, "clawPivot");
        claw = bot.hMap.get(Servo.class, "claw");
    }

    @Override
    public void periodic() {
        clawPivot.setPosition(currentPivotPosition);
        claw.setPosition(currentClawState);
    }

    public void setPosition(double position){
        currentPivotPosition = position;
    }

    public void setState(double state){
        currentClawState = state;
    }

    public boolean getState(){
        if (currentClawState == ClawStates.close){
            return false;
        } else {
            return true;
        }
    }

    public boolean inTolerance(){
        if(Math.abs(clawPivot.getPosition() - currentPivotPosition) < treshold){
            return true;
        }
        return false;
    }

}