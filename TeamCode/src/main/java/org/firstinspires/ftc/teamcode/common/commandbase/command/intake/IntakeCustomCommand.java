package org.firstinspires.ftc.teamcode.common.commandbase.command.intake;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Intake;


/**
 * ManualExtensionCommand is a command that allows the driver to manually control the extension
 * Positive speed extends the extension, negative speed retracts the extension
 */
public class IntakeCustomCommand extends CommandBase {
    private double cspeed;
    private final Intake intake;
    public IntakeCustomCommand(Intake intake, double cspeed) {
        this.intake = intake;
        this.cspeed = cspeed;
    }

    @Override
    public void initialize() {
        intake.setSpeed(cspeed);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
