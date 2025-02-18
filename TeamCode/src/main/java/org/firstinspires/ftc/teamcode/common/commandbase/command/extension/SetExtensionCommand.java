package org.firstinspires.ftc.teamcode.common.commandbase.command.extension;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.Config;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.ClawServo;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Extension;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Pivot;

/**
 * SetExtensionCommand is a command that sets the extension to a specific setpoint
 * Setpoint is in centimeters and the extension is bound to extension limits
 */
public class SetExtensionCommand extends CommandBase {

    private final Extension extension;
    private final double setpoint;
    private final ClawServo claw;

    public SetExtensionCommand(Extension e, ClawServo claw, double setpointcm) {
        extension = e;
        this.setpoint = setpointcm;
        this.claw = claw;
        extension.setSetpointCM(setpoint);
        addRequirements(extension);
    }

    @Override
    public void initialize() {
        new ConditionalCommand(
                new SequentialCommandGroup(
                        new InstantCommand(()->{
                            //claw.setPosition(1);
                        }),
                        new WaitCommand(500),
                        new InstantCommand(()->{
                            extension.setSetpointCM(setpoint);
                        })
                ),
                new SequentialCommandGroup(
                        new InstantCommand(()->{
                            extension.setSetpointCM(setpoint);
                        })
                ),
                ()->claw.clawPivot.getPosition()!= ClawServo.ServoPositions.safe2

        ).schedule();
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(extension.getPositionCM()) - setpoint) <= Config.extension_tolerance;
    }
}
