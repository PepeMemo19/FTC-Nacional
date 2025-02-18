package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import static org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Extension.depositMaxExtension;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.common.Bot;
import org.firstinspires.ftc.teamcode.common.Config;
import org.firstinspires.ftc.teamcode.common.hardware.AbsoluteAnalogEncoder;

@com.acmerobotics.dashboard.config.Config
public class Pivot extends SubsystemBase {

    private final Bot bot;

    public final DcMotor pivotMotorL;
    public final DcMotor pivotMotorR;

    public static final double setpoint_intaking = 88, setpoint_vertical = 0, setpoint_horizontal = 85, setpoint_intaking_start = 80, setpoint_climb = 44;

    private final PIDFController pivotController;
    public double setpointDEG = setpoint_horizontal, minAngle = 0.0, maxAngle = 107;
    private final double encoderOffset = -108.0;

    public Pivot(Bot bot) {
        this.bot = bot;

        pivotMotorL = bot.hMap.get(DcMotor.class, "angleML");
        pivotMotorR = bot.hMap.get(DcMotor.class, "angleMR");

        pivotMotorL.setDirection(DcMotorSimple.Direction.FORWARD);
        pivotMotorR.setDirection(DcMotorSimple.Direction.REVERSE);

        //pivotEncoder = new AbsoluteAnalogEncoder(
        //        bot.hMap.get(AnalogInput.class, "pivotEncoder")
        //);

        pivotMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pivotController = new PIDFController(
                Config.pivot_kP,
                Config.pivot_kI,
                Config.pivot_kD,
                Config.pivot_n_kF
        );
    }

    @Override
    public void periodic() {
        double kFConstant = (Config.pivot_maxH_kF - Config.pivot_h_kF) / depositMaxExtension;
        double extensionFF = (kFConstant * (bot.getExtension().getPositionCM()));
        double pivotFF  = (Math.cos(pivotMotorL.getCurrentPosition() - Math.toRadians(60)));
        double calculatedKf = ((extensionFF + Config.pivot_h_kF) * pivotFF);
        pivotController.setF(calculatedKf);

        double power = pivotController.calculate(
                getPositionDEG(),
                setpointDEG
        );
        pivotMotorR.setPower(-power);
        //pivotMotorL.setPower(power*.30);


        bot.telem.addData("Pivot Angle", getPositionDEG());
        bot.telem.addData("Pivot LPosition", pivotMotorL.getCurrentPosition());
        bot.telem.addData("Pivot RPosition", pivotMotorR.getCurrentPosition());

        bot.telem.addData("Pivot Target", setpointDEG);
        bot.telem.addData("Bot State", bot.getState());
        bot.telem.update();
    }

    /**
     * Set the setpoint of the pivot in degrees
     * @param setpoint the setpoint in degrees
     */
    public void setSetpointDEG(double setpoint) {
        setpointDEG = Math.max(minAngle, Math.min(maxAngle, setpoint));
    }

    /**
     * Set the setpoint of the pivot in degrees, ignoring safety limits
     * USE WITH EXTREME CAUTION, AUTOMATIONS ONLY
     * @param setpoint
     */
    public void setSetpointIGNORE(double setpoint) {
        setpointDEG = setpoint;
    }

    /**
     * Get the setpoint of the pivot in degrees
     * @return the setpoint in degrees
     */
    public double getSetpointDEG() {
        return setpointDEG;
    }

    /**
     * Get the current position of the pivot in degrees
     * @return the position in degrees
     */
    public double getPositionDEG() {
        return (Math.toDegrees(getPositionRAD()) - encoderOffset);
    }

    public double getPositionT() {
        return (pivotMotorL.getCurrentPosition());
    }

    /**
     * Get the current position of the pivot in radians
     * @return the position in radians
     */
    public double getPositionRAD() {
        return ((pivotMotorL.getCurrentPosition() / 8192.0) * 2 * Math.PI);
    }

    public void setSetpointHorizontal() {
        setpointDEG = setpoint_horizontal;
    }

    public void setSetpointVertical() {
        setpointDEG = setpoint_vertical;
    }

    public void multiplyP(){
        pivotController.setP(pivotController.getP()*4);
    }
}