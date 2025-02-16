package org.firstinspires.ftc.teamcode.common.commandbase.command.extension;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.Config;
import org.firstinspires.ftc.teamcode.common.commandbase.command.claw.SetClawPIDCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.ClawPID;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Extension;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Intake;

/**
 * SetExtensionCommand is a command that sets the extension to a specific setpoint
 * Setpoint is in centimeters and the extension is bound to extension limits
 */
public class SetExtensionCommand extends CommandBase {

    private final Extension extension;
    private final double setpoint;
    private final ClawPID claw;

    private final Intake intake;

    private final boolean unsafe;


    public SetExtensionCommand(Extension e, ClawPID claw, Intake intake, double setpointcm, boolean unsafe) {
        extension = e;
        this.setpoint = setpointcm;
        this.claw = claw;
        this.intake = intake;
        this.unsafe = unsafe;
        extension.setSetpointCM(setpoint);
        addRequirements(extension);
    }

    @Override
    public void initialize() {
        new SequentialCommandGroup(
                //new ConditionalCommand(
                //        new SetClawPIDCommand(claw, ClawPID.ServoPositions.safeE),
                //        new WaitCommand(0),
                //        ()->intake.speed!=0
                //),
                new ConditionalCommand(
                        new WaitCommand(0),
                        new ConditionalCommand(
                                new SetClawPIDCommand(claw,ClawPID.ServoPositions.safechambering),
                                new SetClawPIDCommand(claw, ClawPID.ServoPositions.safeE),
                                ()->extension.previousSetpoint==extension.getSamplesTarget()
                        ),
                        ()->unsafe
                ),

                new InstantCommand(()->{
                    extension.setSetpointCM(setpoint);
                })
        ).schedule();
    }

    @Override
    public boolean isFinished() {
        return extension.inTolerance();
    }
}
