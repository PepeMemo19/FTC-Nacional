package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.common.Bot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "PivotF", group = "TeleOp")
public class PivotF extends CommandOpMode {

    private GamepadEx driverGamepad;

    private MultipleTelemetry telem;
    private Bot bot;

    private DcMotor pivotMotorL;
    private DcMotor pivotMotorR;

    @Override
    public void initialize() {
        telem = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //driverGamepad = new GamepadEx(gamepad1);

        pivotMotorL = hardwareMap.get(DcMotor.class, "angleML");
        pivotMotorR = hardwareMap.get(DcMotor.class, "angleMR");

        pivotMotorL.setDirection(DcMotorSimple.Direction.FORWARD);
        pivotMotorR.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()){
            pivotMotorL.setPower(PivotFConfig.pivot_F);
            pivotMotorR.setPower(PivotFConfig.pivot_F);
        }
        pivotMotorL.setPower(0);
        pivotMotorR.setPower(0);
    }

    @com.acmerobotics.dashboard.config.Config
    public static class PivotFConfig {
        public static double pivot_F = -0.004;

    }




}
