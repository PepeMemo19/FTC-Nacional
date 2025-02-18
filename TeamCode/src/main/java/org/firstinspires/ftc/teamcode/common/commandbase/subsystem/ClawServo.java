package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import static org.firstinspires.ftc.teamcode.common.commandbase.subsystem.ClawPID.ServoPIDConfig.d;
import static org.firstinspires.ftc.teamcode.common.commandbase.subsystem.ClawPID.ServoPIDConfig.p;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PDController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.Bot;


@com.acmerobotics.dashboard.config.Config
public class ClawServo extends SubsystemBase {
    private final Bot bot;
    public final Servo clawPivot;
    private double currentPosition = ServoPositions.placing;
    @com.acmerobotics.dashboard.config.Config
    public static class ServoPositions {
        public static double placing = 0.25, safe = 0, intaking = 0.5, safe2= 0;
    }
    public ClawServo(Bot bot) {
        this.bot = bot;
        clawPivot = bot.hMap.get(Servo.class, "clawServo");
    }
    @Override
    public void periodic() {
    }

    public void setPosition(double position){
        clawPivot.setPosition(position);
    }

    //public double getPositionRAD() {
    //    return ((clawEncoder.getCurrentPosition() / 8192.0) * 2 * Math.PI);
    //}

}