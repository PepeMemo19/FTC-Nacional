package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.Bot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "ServoTest", group = "TeleOp")
public class ServoTest extends CommandOpMode {

    private GamepadEx driverGamepad;

    private MultipleTelemetry telem;

    private Servo servo;

    @Override
    public void initialize() {
        telem = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //driverGamepad = new GamepadEx(gamepad1);

        servo = hardwareMap.get(Servo.class, "claw");

        waitForStart();
        while (opModeIsActive()){
            servo.setPosition(ServoTestConfig.position);
        }

    }

    @com.acmerobotics.dashboard.config.Config
    public static class ServoTestConfig {
        public static double position = 1;

    }




}
