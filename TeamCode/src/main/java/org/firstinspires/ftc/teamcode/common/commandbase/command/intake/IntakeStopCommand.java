package org.firstinspires.ftc.teamcode.common.commandbase.command.intake;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Intake;

import java.util.function.DoubleSupplier;


/**
 * ManualExtensionCommand is a command that allows the driver to manually control the extension
 * Positive speed extends the extension, negative speed retracts the extension
 */
public class IntakeStopCommand extends CommandBase {

    private final Intake intake;
    public IntakeStopCommand(Intake intake) {
        this.intake = intake;
    }

    @Override
    public void initialize() {
        intake.stop();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
