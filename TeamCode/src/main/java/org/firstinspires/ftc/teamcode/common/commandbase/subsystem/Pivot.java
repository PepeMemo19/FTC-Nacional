package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import static org.firstinspires.ftc.teamcode.common.Config.pivot_hN_kF;
import static org.firstinspires.ftc.teamcode.common.Config.pivot_h_kF;
import static org.firstinspires.ftc.teamcode.common.Config.pivot_kP;
import static org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Extension.depositMaxExtension;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
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

    public static final double setpoint_intaking = 93, setpoint_vertical = 0, setpoint_horizontal = 87, setpoint_intaking_start = 78, setpoint_climb = 44, setpoint_sampling = 38;

    public final PIDFController pivotController;
    public boolean overrideF = false;
    public double setpointDEG = Pivot.setpoint_horizontal, minAngle = 0.0, maxAngle = 107;
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

        pivotMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void periodic() {
        //double kFConstant = (Config.pivot_maxH_kF - pivot_h_kF) / depositMaxExtension;
        //double extensionFF = (kFConstant * (bot.getExtension().getPositionCM()));
        //double pivotFF  = (Math.cos(pivotMotorL.getCurrentPosition() - Math.toRadians(60)));
        //double calculatedKf = ((extensionFF + pivot_h_kF) * pivotFF);
        //pivotController.setF(calculatedKf);

        double power = pivotController.calculate(
                getPositionDEG(),
                setpointDEG
        );


        if(!overrideF){
            if(bot.opertator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>10) {
                pivotController.setF(pivot_h_kF*2);
            }else if(bot.opertator.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>10) {
                pivotController.setF(pivot_h_kF*-2);
            }else
            if(bot.getExtension().getPositionCM()>1300&&getSetpointDEG()>80){
                pivotController.setF(pivot_h_kF);
                //pivotController.setP(pivot_kP);
            }else if(getSetpointDEG()>70){
                pivotController.setF(pivot_hN_kF);
                //pivotController.setP(pivot_kP);
            }
            else {
                pivotController.setF(0);
            }
        }




        pivotMotorR.setPower(-power);
        //pivotMotorL.setPower(power);


        bot.telem.addData("Pivot InTolerance", inTolerance());
        bot.telem.addData("Pivot TTTTTT", (Math.abs(Math.abs(getPositionDEG()) - Math.abs(getSetpointDEG()))));
        bot.telem.addData("Pivot Angle", getPositionDEG());
        bot.telem.addData("Pivot Setpoint", setpointDEG);
        bot.telem.addData("Pivot F", pivotController.getF());
        bot.telem.addData("Pivot LPosition", pivotMotorL.getCurrentPosition());
        bot.telem.addData("Pivot RPosition", pivotMotorR.getCurrentPosition());
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
     *
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

    public boolean inTolerance(){
        return Math.abs(getPositionDEG() - getSetpointDEG()) <= Config.pivot_tolerance;
    }
}