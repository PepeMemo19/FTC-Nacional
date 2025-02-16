package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.common.Bot;
import org.firstinspires.ftc.teamcode.common.Config;

@com.acmerobotics.dashboard.config.Config
public class ClawPID extends SubsystemBase {
    private final Bot bot;
    public final CRServo clawPivot;
    public final DcMotor clawEncoder;
    public double currentPosition = ServoPositions.placing;
    private PIDFController pd;

    private double angleOffset = 0;
    @com.acmerobotics.dashboard.config.Config
    public static class ServoPositions {
        public static double placing = 80, safeP = 0, safeE = 27, intaking = 250, sampling = 120, specimening = 140,preintaking = 240,safechambering = 170,linear = 0;
    }
    public ClawPID(Bot bot) {
        this.bot = bot;
        clawPivot = bot.hMap.get(CRServo.class, "claw");
        clawPivot.setDirection(DcMotorSimple.Direction.REVERSE);
        clawEncoder = bot.hMap.get(DcMotor.class, "BR");
        pd = new PIDFController(Config.claw_kP, Config.claw_kI, Config.claw_kD, Config.claw_kF);
    }
    @Override
    public void periodic() {
        if(bot.getExtension().getSamplesTarget()>240){
            pd.setF(0.0);
        }else{
            //start pose
            //pd.setF(Config.claw_kF);
        }
        double clawPower = pd.calculate(getPositionDeg(),currentPosition);
        clawPower = MathUtils.clamp(clawPower,-0.9,0.8);
        clawPivot.setPower(clawPower);
        bot.telem.addData("Claw InTolerance",inTolerance());
        bot.telem.addData("Claw Setpoint",currentPosition);
        bot.telem.addData("Claw Position",getPositionDeg());
        bot.telem.addData("Claw Power",clawPivot.getPower());
    }

    public void setPosition(double position){
        currentPosition = position;
    }

    public double getPositionRAD() {
        return ((clawEncoder.getCurrentPosition() / 8192.0) * 2 * Math.PI);
    }

    public double getPositionDeg() {
        return Math.toDegrees(getPositionRAD())-angleOffset;
    }

    public boolean inTolerance(){
        return Math.abs(getPositionDeg()) - Math.abs(currentPosition) < 2;
    }

    public boolean inToleranceFor(double target){
        return Math.abs(getPositionDeg()) - Math.abs(target) < 2;
    }

}