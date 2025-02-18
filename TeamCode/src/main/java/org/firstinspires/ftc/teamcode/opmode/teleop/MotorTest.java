package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.common.Bot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Motor Test", group = "TeleOp")
public class MotorTest extends CommandOpMode {

    private GamepadEx driverGamepad;

    private MultipleTelemetry telem;
    private Bot bot;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        telem = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        driverGamepad = new GamepadEx(gamepad1);

        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "FL");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "FR");
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "BL");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "BR");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        new GamepadButton(driverGamepad, GamepadKeys.Button.Y)
                .whenPressed(
                        new InstantCommand(() -> {
                            frontLeft.setPower(0.5);
                        })
                );

        new GamepadButton(driverGamepad, GamepadKeys.Button.Y).whenReleased(
            new InstantCommand(() -> {
                frontLeft.setPower(0);
            })
        );

        new GamepadButton(driverGamepad, GamepadKeys.Button.B)
                .whenPressed(
                        new InstantCommand(() -> {
                            backLeft.setPower(0.5);
                        })
                );

        new GamepadButton(driverGamepad, GamepadKeys.Button.B).whenReleased(
            new InstantCommand(() -> {
                backLeft.setPower(0);
            })
        );

        new GamepadButton(driverGamepad, GamepadKeys.Button.X)
                .whenPressed(
                        new InstantCommand(() -> {
                            frontRight.setPower(0.5);
                        })
                );

        new GamepadButton(driverGamepad, GamepadKeys.Button.X).whenReleased(
            new InstantCommand(() -> {
                frontRight.setPower(0);
            })
        );

        new GamepadButton(driverGamepad, GamepadKeys.Button.A)
                .whenPressed(
                        new InstantCommand(() -> {
                            backRight.setPower(0.5);
                        })
                );

        new GamepadButton(driverGamepad, GamepadKeys.Button.A).whenReleased(
            new InstantCommand(() -> {
                backRight.setPower(0);
            })
        );


    }




}
