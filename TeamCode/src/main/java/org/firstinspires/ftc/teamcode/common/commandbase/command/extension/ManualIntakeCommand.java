package org.firstinspires.ftc.teamcode.common.commandbase.command.extension;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.Config;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Extension;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Intake;

import java.util.function.DoubleSupplier;


/**
 * ManualExtensionCommand is a command that allows the driver to manually control the extension
 * Positive speed extends the extension, negative speed retracts the extension
 */
public class ManualIntakeCommand extends CommandBase {

    private final Intake intake;
    private final DoubleSupplier in, out;

    /**
     * Constructor for ManualExtensionCommand
     * @param i Extension subsystem
     * @param inModifier Supplier for extension retraction speed (in)
     * @param outModifier Supplier for extension extension speed (out)
     */
    public ManualIntakeCommand(Intake i, DoubleSupplier inModifier, DoubleSupplier outModifier) {
        intake = i;
        in = inModifier;
        out = outModifier;
        addRequirements(intake);
    }

    @Override
    public void execute() {
       intake.speed = in.getAsDouble() - out.getAsDouble();
    }
}
