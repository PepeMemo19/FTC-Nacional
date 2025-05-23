package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.common.Bot;
import org.firstinspires.ftc.teamcode.common.Config;

@com.acmerobotics.dashboard.config.Config
public class Extension extends SubsystemBase {

    private final Bot bot;

    public final DcMotor bottomExtensionMotor, topExtensionMotor;

    private final PIDFController extensionController;
    public static double setpointCM = 0.0, highBarTarget = 1070, lowBarTarget = 17.0, lowBasketTarget = 1500.0, highBasketTarget = 3250, ticksperCM = 1, climb = 2100, climb2= 640; //10.37339803;
    public double previousSetpoint = 0;
    public static double minExtension = 0.0, depositMaxExtension = 30, intakeMaxExtension = 1800, specimening = 550;

    public static double samples2 = 75;
    public double getBarTarget() {
        if (bot.getLevel() == Bot.Levels.Up) {
            return highBarTarget;
        } else {
            return lowBarTarget;
        }
    }

    public double getSamplesTarget() {
        if (bot.getLevel() == Bot.Levels.Up) {
            return highBasketTarget;
        } else {
            return lowBasketTarget;
        }
    }

    public Extension(Bot bot) {
        this.bot = bot;

        bottomExtensionMotor = bot.hMap.get(DcMotor.class, "viperL");
        bottomExtensionMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomExtensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomExtensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        topExtensionMotor = bot.hMap.get(DcMotor.class, "viperR");
        topExtensionMotor.setDirection(DcMotor.Direction.FORWARD);
        topExtensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topExtensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        extensionController = new PIDFController(
                Config.extension_kP,
                Config.extension_kI,
                Config.extension_kD,
                Config.extension_kF
        );
        //extensionController.setTolerance(Config.extension_tolerance);
    }

    @Override
    public void periodic() {
        double calculatedkF = Config.extension_kF * (Math.cos(bot.getPivot().getPositionRAD() - Math.toRadians(90)));
        extensionController.setF(calculatedkF);

        double target = setpointCM;

        double power = extensionController.calculate(
                topExtensionMotor.getCurrentPosition(),
                target * ticksperCM
        );

        topExtensionMotor.setPower(power);
        bottomExtensionMotor.setPower(power);

        bot.telem.addData("Extension InTolerance", inTolerance());
        bot.telem.addData("Extension Position", getPositionCM());
        bot.telem.addData("Extension Target", getSetpointCM());
        bot.telem.addData("Calculated Extension kF", extensionController.getF());
    }

    /**
     * Set the setpoint for the extension in centimeters
     * @param setpoint the setpoint in centimeters
     */
    public void setSetpointCM(double setpoint) {
        previousSetpoint = setpointCM;
        setpointCM = setpoint;
    }

    /**
     * Get the setpoint for the extension in centimeters
     * @return the current setpoint of the extension in centimeters
     */
    public double getSetpointCM() {
        return setpointCM;
    }

    /**
     * Get the current position of the extension in centimeters
     * @return the current position of the extension in centimeters
     */
    public double getPositionCM() {
        return topExtensionMotor.getCurrentPosition() / ticksperCM;
    }

    /**
     * Get the maximum extension of the extension in centimeters
     * @return the maximum extension of the extension in centimeters
     */
    public double getMaxExtension() {
        return intakeMaxExtension;
    }

    /**
     * Get the minimum extension of the extension in centimeters
     * @return the minimum extension of the extension in centimeters
     */
    public double getMinExtension() {
        return minExtension;
    }

    public void multiplyP(){
        extensionController.setP(extensionController.getP()*4);
    }
    public void resetP(){
        extensionController.setP(Config.extension_kP);
    }

    public boolean inTolerance(){
        return (Math.abs(getPositionCM()) - setpointCM) <= Config.extension_tolerance;
    }

}
