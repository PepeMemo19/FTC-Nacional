package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import static org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Extension.depositMaxExtension;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.common.Bot;
import org.firstinspires.ftc.teamcode.common.Config;

@com.acmerobotics.dashboard.config.Config
public class Intake extends SubsystemBase {

    private final Bot bot;

    public final CRServo roller1;
    public final CRServo roller2;

    public double speed;

    public Intake(Bot bot) {
        this.bot = bot;

        roller1 = bot.hMap.get(CRServo.class, "rollerL");
        roller2 = bot.hMap.get(CRServo.class, "rollerR");
    }

    @Override
    public void periodic() {
        roller1.setPower(-speed);
        roller2.setPower(speed);
    }

    public void in(){
        speed = 1;
    }

    public void out(){
        speed = -1;
    }

    public void stop(){
        speed = 0;
    }

}