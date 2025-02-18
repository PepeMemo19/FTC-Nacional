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
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.BotState;
import org.firstinspires.ftc.teamcode.common.Bot;
import org.firstinspires.ftc.teamcode.common.commandbase.command.claw.SetClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.drive.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.extension.ManualExtensionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.extension.ManualIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.extension.SetExtensionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.intake.IntakeInCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.intake.IntakeOutCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.intake.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.pivot.ManualPivotCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.pivot.SetPivotAngleCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.state.SetBotStateCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.ClawServo;
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
    private ClawServo claw;
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
                () -> driverGamepad.getRightX(),
                () -> -driverGamepad.getLeftY(),
                () -> driverGamepad.getLeftX(),
                () -> bot.speed
        );


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

        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_UP)
                .whileHeld(
                        new InstantCommand(()->{
                            drivetrain.setTargetHeadingDEG(0);
                            drivetrain.setHeadingLock(true);
                        })
                );

        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_RIGHT)
                .whileHeld(
                        new InstantCommand(()->{
                            drivetrain.setTargetHeadingDEG(transformangle);
                            drivetrain.setHeadingLock(true);
                        })
                );

        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_DOWN)
                .whileHeld(
                        new InstantCommand(()->{
                            drivetrain.setTargetHeadingDEG(180);
                            drivetrain.setHeadingLock(true);
                        })
                );

        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_LEFT)
                .whileHeld(
                        new InstantCommand(()->{
                            drivetrain.setTargetHeadingDEG(basketAngle);
                            drivetrain.setHeadingLock(true);
                        })
                );

        if(driverGamepad != operatorGamepad){
            new GamepadButton(driverGamepad, GamepadKeys.Button.Y)
                    .whileHeld(
                            new InstantCommand(()->{
                                drivetrain.setTargetHeadingDEG(0);
                                drivetrain.setHeadingLock(true);
                            })
                    ).whenReleased(
                            new  InstantCommand(()->{
                                drivetrain.setHeadingLock(false);
                            })
                    );

            new GamepadButton(driverGamepad, GamepadKeys.Button.B)
                    .whileHeld(
                            new InstantCommand(()->{
                                drivetrain.setTargetHeadingDEG(transformangle);
                                drivetrain.setHeadingLock(true);
                            })
                    ).whenReleased(
                            new  InstantCommand(()->{
                                drivetrain.setHeadingLock(false);
                            })
                    );

            new GamepadButton(driverGamepad, GamepadKeys.Button.A)
                    .whileHeld(
                            new InstantCommand(()->{
                                drivetrain.setTargetHeadingDEG(180);
                                drivetrain.setHeadingLock(true);
                            })
                    ).whenReleased(
                            new  InstantCommand(()->{
                                drivetrain.setHeadingLock(false);
                            })
                    );

            new GamepadButton(driverGamepad, GamepadKeys.Button.X)
                    .whileHeld(
                            new InstantCommand(()->{
                                drivetrain.setTargetHeadingDEG(basketAngle);
                                drivetrain.setHeadingLock(true);
                            })
                    ).whenReleased(
                            new  InstantCommand(()->{
                                drivetrain.setHeadingLock(false);
                            })
                    );
        }

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

        /*
        ManualExtensionCommand extensionCommand = new ManualExtensionCommand(
                extension,
                () -> operatorGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER),
                () -> operatorGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)
        );
        */

        ManualIntakeCommand intakeCommand = new ManualIntakeCommand(
                intake,
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
                            if(pivot.setpointDEG==Pivot.setpoint_horizontal){
                                new SetPivotAngleCommand(pivot, claw,Pivot.setpoint_vertical);
                            }else {
                                new SetPivotAngleCommand(pivot,claw,Pivot.setpoint_horizontal);
                            }
                        }
                        )
                );

        register(extension);
        //extension.setDefaultCommand(extensionCommand);
        intake.setDefaultCommand(intakeCommand);
        //endregion

        //region Actions


        new GamepadButton(operatorGamepad, GamepadKeys.Button.B)
                .whileActiveOnce(
                        new SequentialCommandGroup(
                                new SetPivotAngleCommand(pivot, claw, Pivot.setpoint_intaking),
                                //new IntakeInCommand(intake),
                                new SetClawCommand(claw, ClawServo.ServoPositions.intaking),
                                new SetExtensionCommand(extension, claw, Extension.intakeMaxExtension)
                        )
                );

        new GamepadButton(operatorGamepad, GamepadKeys.Button.B)
                .whenReleased(
                        new SequentialCommandGroup(
                                new SetPivotAngleCommand(pivot,claw,Pivot.setpoint_horizontal),
                                new SetClawCommand(claw, ClawServo.ServoPositions.safe),
                                new WaitCommand(300),
                                new SetExtensionCommand(extension,  claw, 0)
                        )
                );

        //new GamepadButton(operatorGamepad, GamepadKeys.Button.B)
        //        .whenReleased(
        //            new SequentialCommandGroup(
        //                    new SetExtensionCommand(extension, claw, 0)
        //            )/*.interruptOn(() -> driverGamepad.getButton(GamepadKeys.Button.B))*/
        //        );


        //Define Y Action
        new GamepadButton(operatorGamepad, GamepadKeys.Button.Y)
                .whenPressed(
                        new ConditionalCommand(
                                //if intaking
                                new WaitCommand(0),
                                new ConditionalCommand(
                                        //if depositing
                                        new ConditionalCommand(
                                                //Put Specimen
                                                new SequentialCommandGroup(
                                                        new SetExtensionCommand(extension, claw, extension.getSetpointCM()-300),
                                                        new IntakeOutCommand(intake),
                                                        new WaitCommand(5000),
                                                        new IntakeStopCommand(intake)
                                                ),
                                                //Put Sample
                                                new SequentialCommandGroup(
                                                        new IntakeOutCommand(intake),
                                                        new WaitCommand(700),
                                                        new SetClawCommand(claw,ClawServo.ServoPositions.safe2),
                                                        new IntakeStopCommand(intake),
                                                        new WaitCommand(200),
                                                        new SetExtensionCommand(extension, claw, 0)
                                                ),
                                                () -> bot.getMode() == Bot.Modes.SPECIMENS
                                        ),
                                        //if not intaking or depositing
                                        new WaitCommand(0),
                                        () -> driverGamepad.getButton(GamepadKeys.Button.X)
                                ),
                                () -> driverGamepad.getButton(GamepadKeys.Button.B)
                        )

                );

        //start deposit sample/specimen
        new GamepadButton(operatorGamepad, GamepadKeys.Button.X)
                .whenPressed(
                        new ConditionalCommand(
                                //Specimens
                                new SequentialCommandGroup(
                                        new SetClawCommand(claw, ClawServo.ServoPositions.safe),
                                        new SetPivotAngleCommand(pivot, claw, Pivot.setpoint_vertical),
                                        new SetExtensionCommand(extension, claw, extension.getBarTarget()),
                                        new SetClawCommand(claw, ClawServo.ServoPositions.placing)
                                ),
                                //Samples
                                new SequentialCommandGroup(
                                        new SetClawCommand(claw, ClawServo.ServoPositions.safe),
                                        new SetPivotAngleCommand(pivot, claw, Pivot.setpoint_vertical),
                                        new SetExtensionCommand(extension, claw, extension.getSamplesTarget()),
                                        //new WaitCommand(5000),
                                        new SetClawCommand(claw, ClawServo.ServoPositions.placing)
                                ),
                                () -> bot.getMode() == Bot.Modes.SPECIMENS
                        )

                );

        //extension release
        new GamepadButton(operatorGamepad, GamepadKeys.Button.X)
                .whenReleased(
                        new SequentialCommandGroup(
                                new SetClawCommand(claw, ClawServo.ServoPositions.safe),
                                new SetExtensionCommand(extension, claw, 0)
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

        new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand(()->{
                    bot.setLevel(Bot.Levels.Up);
                })
        );

        new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new InstantCommand(()->{
                    bot.setLevel(Bot.Levels.Down);
                })
        );

        //endregion

        //region Climb
        new GamepadButton(operatorGamepad, GamepadKeys.Button.LEFT_STICK_BUTTON).whileActiveOnce(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new SetExtensionCommand(extension, claw, 1325),
                                new SetPivotAngleCommand(pivot,claw,39)
                        ),
                        new WaitUntilCommand(()->operatorGamepad.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)),
                        new InstantCommand(()->{
                            bot.climbing = true;
                        }),
                        new InstantCommand(()->{
                            // pivot.multiplyP();
                        }),
                        new SetPivotAngleCommand(pivot,claw, Pivot.setpoint_climb),
                        new ParallelCommandGroup(
                                new InstantCommand(()->{
                                    //extension.multiplyP();
                                }),
                                new SequentialCommandGroup(
                                        new SetExtensionCommand(extension, claw, -5),
                                        new InstantCommand(()->{
                                            //extension.multiplyP();
                                        })
                                ),

                                new SetPivotAngleCommand(pivot,claw,110)
                        )

                )
        );

        new GamepadButton(operatorGamepad, GamepadKeys.Button.LEFT_STICK_BUTTON).whenReleased(
                new ConditionalCommand(
                        new WaitCommand(0),
                        new ParallelCommandGroup(
                                new SetExtensionCommand(extension, claw, 1),
                                new SetPivotAngleCommand(pivot,claw,Pivot.setpoint_horizontal)
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
                        new SetBotStateCommand(bot, BotState.INTAKE),
                        new SetExtensionCommand(bot.getExtension(), claw,  0),
                        new SetPivotAngleCommand(bot.getPivot(), claw, Pivot.setpoint_horizontal),
                        new SetClawCommand(bot.getClaw(),ClawServo.ServoPositions.safe)

                )
        );

        while (opModeInInit()){
            bot.telem.addData("Pivot Angle", pivot.getPositionDEG());
            //bot.telem.addData("Pivot AngleT", pivot.getPositionT());
            //bot.telem.addData("Pivot LPosition", pivot.pivotMotorL.getCurrentPosition());
            //bot.telem.addData("Pivot RPosition", pivot.pivotMotorR.getCurrentPosition());
            bot.telem.addData("ViperL", extension.bottomExtensionMotor.getCurrentPosition());
            bot.telem.addData("ViperR", extension.topExtensionMotor.getCurrentPosition());
            bot.telem.update();
        }

        //endregion


    }

}