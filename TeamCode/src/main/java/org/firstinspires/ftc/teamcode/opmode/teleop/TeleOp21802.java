package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.common.Bot;
import org.firstinspires.ftc.teamcode.common.commandbase.command.drive.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.state.SetBotStateCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.state.SetGamepadLight;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.ClawServo;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Extension;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Intake;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Pivot;

public class TeleOp21802 extends CommandOpMode {
    private Bot bot;
    private Pivot pivot;
    private Extension extension;
    private Intake intake;
    private ClawServo claw;
    private MecanumDrivetrain drivetrain;
    private boolean enableDrive = true;

    private GamepadEx driverGamepad;
    private GamepadEx mechanismGamepad;

    private MultipleTelemetry telem;

    @Override
    public void initialize(){

        //region Initialize

        CommandScheduler.getInstance().reset();

        telem = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        driverGamepad = new GamepadEx(gamepad1);
        //operatorGamepad = driverGamepad;
        mechanismGamepad = new GamepadEx(gamepad2);

        //gamepad1.setLedColor(255, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);

        bot = new Bot(telem, hardwareMap, driverGamepad, mechanismGamepad, enableDrive);

        register(drivetrain);

        intake = bot.getIntake();
        claw = bot.getClaw();
        register(intake);
        register(claw);

        //endregion

        //region Drivetrain
        drivetrain = bot.getDrivetrain();

        TeleOpDriveCommand driveCommand = new TeleOpDriveCommand(
                drivetrain,
                () -> driverGamepad.getRightX(),
                () -> -driverGamepad.getLeftY(),
                () -> driverGamepad.getLeftX(),
                () -> bot.speed
        );
        drivetrain.setDefaultCommand(driveCommand);

        //endregion

        //region Buttons
        GamepadKeys.Button Change_mode = GamepadKeys.Button.BACK;
        GamepadKeys.Button climb = GamepadKeys.Button.BACK;

        GamepadKeys.Button horizontal = GamepadKeys.Button.BACK;
        GamepadKeys.Trigger Add_extension = GamepadKeys.Trigger.LEFT_TRIGGER;
        GamepadKeys.Trigger Remove_extension = GamepadKeys.Trigger.RIGHT_TRIGGER;
        GamepadKeys.Button high = GamepadKeys.Button.DPAD_UP;
        GamepadKeys.Button low = GamepadKeys.Button.DPAD_DOWN;

        GamepadKeys.Button Put_down = GamepadKeys.Button.B;
        GamepadKeys.Button score = GamepadKeys.Button.A;

        GamepadKeys.Button Floor_pickup = GamepadKeys.Button.Y;
        GamepadKeys.Button Specimen_pickup = GamepadKeys.Button.X;
        //endregion

        //region SetMode
        new GamepadButton(mechanismGamepad, Change_mode)
                .whenPressed(
                        new ParallelCommandGroup(
                            new SetBotStateCommand(bot),
                            new SetGamepadLight(bot, mechanismGamepad)
                        )
                );

        //endregion

        //region Pickup


        //endregion

        //region Specimen


        //endregion

        //region Sample


        //endregion

        //region Climber


        //endregion

    }
}
