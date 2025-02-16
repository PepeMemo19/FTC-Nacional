package org.firstinspires.ftc.teamcode.common.commandbase.command.claw;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Claw;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.ClawPID;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Intake;


/**
 * ManualExtensionCommand is a command that allows the driver to manually control the extension
 * Positive speed extends the extension, negative speed retracts the extension
 */
public class SetClawPIDCommand extends CommandBase {

    private final ClawPID claw;
    private final double position;

    public SetClawPIDCommand(ClawPID claw, double position) {
        this.claw = claw;
        this.position = position;
    }

    @Override
    public void initialize() {
        claw.setPosition(position);
    }

    @Override
    public boolean isFinished() {
        return claw.inTolerance();
    }
}
