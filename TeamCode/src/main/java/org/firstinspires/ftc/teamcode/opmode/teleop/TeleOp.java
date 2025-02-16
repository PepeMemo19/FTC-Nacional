package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.common.Bot;
import org.firstinspires.ftc.teamcode.common.commandbase.command.actions.EndIntake2Command;
import org.firstinspires.ftc.teamcode.common.commandbase.command.actions.EndIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.actions.GrabSpecimen;
import org.firstinspires.ftc.teamcode.common.commandbase.command.actions.PutSample;
import org.firstinspires.ftc.teamcode.common.commandbase.command.actions.PutSpecimen;
import org.firstinspires.ftc.teamcode.common.commandbase.command.actions.PutSpecimen2;
import org.firstinspires.ftc.teamcode.common.commandbase.command.actions.PutSpecimen2v2;
import org.firstinspires.ftc.teamcode.common.commandbase.command.actions.StartDepositSample;
import org.firstinspires.ftc.teamcode.common.commandbase.command.actions.StartDepositSpecimen;
import org.firstinspires.ftc.teamcode.common.commandbase.command.actions.StartDepositSpecimen2;
import org.firstinspires.ftc.teamcode.common.commandbase.command.actions.StartDepositSpecimen2v2;
import org.firstinspires.ftc.teamcode.common.commandbase.command.actions.StartIntake2Command;
import org.firstinspires.ftc.teamcode.common.commandbase.command.actions.StartIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.claw.SetClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.claw.SetClawPIDCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.drive.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.extension.ManualExtensionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.extension.SetExtensionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.intake.IntakeCustomCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.intake.IntakeInCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.intake.IntakeOutCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.intake.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.pivot.ManualPivotCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.pivot.SetPivotAngleCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.ClawPID;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Extension;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Intake;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Pivot;
import org.firstinspires.ftc.teamcode.Direction;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "TeleOp")
public class TeleOp extends CommandOpMode {

    private Bot bot;
    private Pivot pivot;
    private Extension extension;
    private Intake intake;
    private ClawPID claw;
    private MecanumDrivetrain drivetrain;
    private boolean enableDrive = true;

    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;

    private MultipleTelemetry telem;

    // Gamepad layout
    // https://www.padcrafter.com/?templates=Gamepad+1%2FDriver+Gamepad&plat=1&col=%23242424%2C%23606A6E%2C%23FFFFFF&rightStick=Yaw%2FRotation&leftStick=Translation&dpadUp=Wrist+Up&dpadRight=Wrist+Clockwise&dpadLeft=Wrist+Counter-Clockwise&dpadDown=Wrist+Down&aButton=Toggle+Claw&yButton=Sample%2FSpecimen+Auto+Deposit&xButton=Robot+State+Toggle&bButton=Sample%2FSpecimen+State+Toggle&rightTrigger=Extension+Out&leftTrigger=Extension+In&leftBumper=Pivot+Down&rightBumper=Pivot+Up&backButton=Red+%3D+Sample%2C+Blue+%3D+Specimen%2C+Green+%3D+Ascent&startButton=Options+%3D+Ascent    @Override
    public void initialize() {


        CommandScheduler.getInstance().reset();

        telem = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());



        driverGamepad = new GamepadEx(gamepad1);
        //operatorGamepad = driverGamepad;
        operatorGamepad = new GamepadEx(gamepad2);

        //gamepad1.setLedColor(255, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);

        bot = new Bot(telem, hardwareMap, driverGamepad, operatorGamepad, enableDrive);

        //region Drivetrain
        drivetrain = bot.getDrivetrain();

        TeleOpDriveCommand driveCommand = new TeleOpDriveCommand(
                drivetrain,
                () -> driverGamepad.getRightX()*.80,
                () -> -driverGamepad.getLeftY()*1,
                () -> driverGamepad.getLeftX()*1,
                () -> bot.speed
        );
        bot.speed = 0.75;





//driverGamepad.getTrigger(driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)).

        //Button fieldCentricToggle = (new GamepadButton(driverGamepad, GamepadKeys.Button.BACK))
        //        .whenPressed(
        //                new ToggleFieldCentricCommand(bot.getDrivetrain())
        //        );



        register(drivetrain);
        drivetrain.setDefaultCommand(driveCommand);

        intake = bot.getIntake();
        claw = bot.getClaw();
        register(intake);
        register(claw);

        //region HLock
        double basketAngle = 180+45;
        double transformangle = -45;

        //endregion

        //region Pivot
        pivot = bot.getPivot();

        Button pivotDownButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_LEFT))
                .whenPressed(
                        new ManualPivotCommand(pivot, Direction.DOWN)
                );
        Button pivotUpButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_RIGHT))
                .whenPressed(
                        new ManualPivotCommand(pivot, Direction.UP)
                );

        register(pivot);
        //endregion

        //region Extension
        extension = bot.getExtension();

        ManualExtensionCommand extensionCommand = new ManualExtensionCommand(
                extension,
                () -> operatorGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER),
                () -> operatorGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)
        );

        //Button toggleElevator = (new GamepadButton(driverGamepad, GamepadKeys.Button.A))
        //        .whenPressed(
        //                new InstantCommand(()->{
        //                            if(pivot.setpointDEG==0){
        //                                pivot.setSetpointHorizontal();
        //                            }else {
        //                                pivot.setSetpointVertical();
        //                            }
        //                        }
        //                )
        //        );

        Button togglePivot = (new GamepadButton(operatorGamepad, GamepadKeys.Button.A))
                .whenPressed(
                        new InstantCommand(()->{
                            if(pivot.setpointDEG==Pivot.setpoint_vertical){
                                new SetPivotAngleCommand(pivot, claw, Pivot.setpoint_horizontal);
                            }else {
                                new SetPivotAngleCommand(pivot,claw, Pivot.setpoint_vertical);
                            }
                        }
                        )
                );

        register(extension);
        extension.setDefaultCommand(extensionCommand);
        //endregion

        //region Actions

        new GamepadButton(operatorGamepad, GamepadKeys.Button.B)
                .whenReleased(
                        new SequentialCommandGroup(
                                new SetExtensionCommand(extension,  claw,intake, 0,true),
                                new SetClawPIDCommand(claw, ClawPID.ServoPositions.placing)
                        )
                );


        new GamepadButton(operatorGamepad, GamepadKeys.Button.B)
                .whileActiveOnce(
                        new StartIntakeCommand(bot)
                );

        //Define Y Action
        new GamepadButton(operatorGamepad, GamepadKeys.Button.Y)
                .whenPressed(
                        new ConditionalCommand(
                                //if intaking
                                new EndIntakeCommand(bot),
                                new WaitCommand(0),
                                () -> operatorGamepad.getButton(GamepadKeys.Button.B)
                        )

                );


        //Define Y Action
        new GamepadButton(operatorGamepad, GamepadKeys.Button.Y)
                .whenActive(
                        new ConditionalCommand(
                                //if intaking
                                new WaitCommand(0),
                                new ConditionalCommand(
                                        //if depositing
                                        new ConditionalCommand(
                                                //Put Specimen
                                                new PutSpecimen2v2(bot),
                                                //Put Sample
                                                new PutSample(bot),
                                                () -> bot.getMode() == Bot.Modes.SPECIMENS
                                        ),
                                        //if not intaking or depositing
                                        new WaitCommand(0),
                                        () -> operatorGamepad.getButton(GamepadKeys.Button.X)
                                ),
                                () -> operatorGamepad.getButton(GamepadKeys.Button.B)
                        )

                );

        //start deposit sample/specimen
        new GamepadButton(operatorGamepad, GamepadKeys.Button.X)
                .whileActiveOnce(
                        new ConditionalCommand(
                                //Specimens
                                new StartDepositSpecimen2v2(bot),
                                //Samples
                                new StartDepositSample(bot),
                                () -> bot.getMode() == Bot.Modes.SPECIMENS
                        )

                );//2 derecha

        //extension release
        new GamepadButton(operatorGamepad, GamepadKeys.Button.X)
                .whenReleased(
                        new SequentialCommandGroup(
                                new SetExtensionCommand(extension, claw,intake, 0,false)
                        )/*.interruptOn(() -> driverGamepad.getButton(GamepadKeys.Button.X))*/
                );


        //endregion

        //region Intake

        //endregion

        //region Settings

        new GamepadButton(operatorGamepad, GamepadKeys.Button.BACK).whenPressed(
                new InstantCommand(()->{
                    bot.toggleMode();
                })
        );

        new GamepadButton(driverGamepad, GamepadKeys.Button.BACK).whenPressed(
                new InstantCommand(()->{
                    bot.getDrivetrain().odo.resetPosAndIMU();
                })
        );

        //new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_UP).whenPressed(
        //        new InstantCommand(()->{
        //            bot.setLevel(Bot.Levels.Up);
        //        })
        //);

        //new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_DOWN).whenPressed(
        //        new InstantCommand(()->{
        //            bot.setLevel(Bot.Levels.Down);
        //        })
        //);
        new GamepadButton(operatorGamepad, GamepadKeys.Button.RIGHT_STICK_BUTTON).whileActiveOnce(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new SetExtensionCommand(extension, claw,intake, 640,false),
                                new WaitCommand (8000),
                                new SetPivotAngleCommand(pivot,claw,80)
                        )
                )
        );




        //endregion

        //region Climb
        new GamepadButton(operatorGamepad, GamepadKeys.Button.LEFT_STICK_BUTTON).whileActiveOnce(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new SetExtensionCommand(extension, claw,intake, 2100,false),
                                new SetPivotAngleCommand(pivot,claw,30)
                        ),
                        new WaitUntilCommand(()->operatorGamepad.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)),
                        new InstantCommand(()->{
                            bot.climbing = true;
                        }),
                        new IntakeStopCommand(bot.getIntake()),
                        new InstantCommand(()->{
                            // pivot.multiplyP();
                        }),
                        new SetPivotAngleCommand(pivot,claw, Pivot.setpoint_climb),
                        new ParallelCommandGroup(
                                new InstantCommand(()->{
                                    //extension.multiplyP();
                                }),
                                new SequentialCommandGroup(
                                        new SetExtensionCommand(extension, claw,intake, -5,false),
                                        new InstantCommand(()->{
                                            //extension.multiplyP();
                                        })
                                ),

                                new SetPivotAngleCommand(pivot,claw,110)
                        )

                )
        );


        new GamepadButton(operatorGamepad,GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new SequentialCommandGroup(
                        new SetPivotAngleCommand(pivot,claw,Pivot.setpoint_vertical),
                        new SetClawPIDCommand(claw,ClawPID.ServoPositions.placing),
                        new IntakeOutCommand(intake)
                )
        );

        new GamepadButton(operatorGamepad,GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new SequentialCommandGroup(
                        new GrabSpecimen(bot)
                        //new WaitUntilCommand(()->!operatorGamepad.getButton(GamepadKeys.Button.LEFT_BUMPER))
                        //new SetClawPIDCommand(claw,)

                )
        );

        new GamepadButton(operatorGamepad, GamepadKeys.Button.LEFT_STICK_BUTTON).whenReleased(
                new ConditionalCommand(
                        new WaitCommand(0),
                        new ParallelCommandGroup(
                                new SetExtensionCommand(extension, claw,intake, 1,false),
                                new SetPivotAngleCommand(pivot,claw,1)
                        ),
                        () -> bot.climbing
                )
        );

        //endregion

        //region Test
        //new GamepadButton(operatorGamepad, GamepadKeys.Button.LEFT_BUMPER).whileHeld(
        //        new InstantCommand(()->{intake.in();})
        //).whenReleased(
        //        new InstantCommand(()->{intake.stop();})
        //);

        //new GamepadButton(operatorGamepad, GamepadKeys.Button.RIGHT_BUMPER).whileHeld(
        //        new InstantCommand(()->{intake.out();})
        //).whenReleased(
        //        new InstantCommand(()->{intake.stop();})
        //);
        //endregion

        //region Automation


        schedule(
                new ParallelCommandGroup(
                        //new SetBotStateCommand(bot, BotState.INTAKE),
                        //new SetExtensionCommand(bot.getExtension(), claw,  0),
                        //new SetPivotAngleCommand(bot.getPivot(), claw, Pivot.setpoint_horizontal),
                        new SetClawPIDCommand(claw,30)
                )
        );

        while (opModeInInit()){
            bot.telem.addData("Pivot Angle", pivot.getPositionDEG());
            //bot.telem.addData("Pivot AngleT", pivot.getPositionT());
            //bot.telem.addData("Pivot LPosition", pivot.pivotMotorL.getCurrentPosition());
            //bot.telem.addData("Pivot RPosition", pivot.pivotMotorR.getCurrentPosition());
            bot.telem.addData("ViperL", extension.bottomExtensionMotor.getCurrentPosition());
            bot.telem.addData("ViperR", extension.topExtensionMotor.getCurrentPosition());
            bot.telem.addData("Claw Setpoint",claw.currentPosition);
            bot.telem.addData("Claw Position",claw.getPositionDeg());
            bot.telem.update();
        }

        //endregion


    }

}