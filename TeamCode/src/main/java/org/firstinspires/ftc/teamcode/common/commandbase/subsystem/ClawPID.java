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
public class ClawPID extends SubsystemBase {
    private final Bot bot;
    public final CRServo clawPivot;
    public final DcMotor clawEncoder;
    private double currentPosition = ServoPositions.placing;
    private PDController pd;
    @com.acmerobotics.dashboard.config.Config
    public static class ServoPositions {
        public static double placing = 0.6, safe = 0.3, intaking = 0.15, safe2= 0.15;
    }

    @com.acmerobotics.dashboard.config.Config
    public static class ServoPIDConfig {
        public static double p = 0;
        public static double d = 0;
    }
    public ClawPID(Bot bot) {
        this.bot = bot;
        clawPivot = bot.hMap.get(CRServo.class, "claw");
        clawEncoder = bot.hMap.get(DcMotor.class, "encoder");
        clawEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pd = new PDController(p, d);
    }
    @Override
    public void periodic() {
        clawPivot.setPower(pd.calculate(currentPosition,getPositionRAD()));
    }

    public void setPosition(double position){
        currentPosition = position;
    }

    public double getPositionRAD() {
        return ((clawEncoder.getCurrentPosition() / 8192.0) * 2 * Math.PI);
    }

}