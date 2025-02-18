package org.firstinspires.ftc.teamcode.common.commandbase.command.claw;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Claw;


/**
 * ManualExtensionCommand is a command that allows the driver to manually control the extension
 * Positive speed extends the extension, negative speed retracts the extension
 */
public class SetClawPIDCommand extends CommandBase {

    private final Claw claw;
    private final double position;

    public SetClawPIDCommand(Claw claw, double position) {
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
