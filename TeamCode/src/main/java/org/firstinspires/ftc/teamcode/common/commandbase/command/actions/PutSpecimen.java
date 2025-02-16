package org.firstinspires.ftc.teamcode.common.commandbase.command.actions;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.common.Bot;
import org.firstinspires.ftc.teamcode.common.commandbase.command.claw.SetClawPIDCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.extension.SetExtensionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.intake.IntakeOutCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.intake.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.pivot.SetPivotAngleCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.ClawPID;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Pivot;

public class PutSpecimen extends SequentialCommandGroup {
    public PutSpecimen(Bot b){
        addCommands(
                new SetClawPIDCommand(b.getClaw(), ClawPID.ServoPositions.placing),
                new SetExtensionCommand(b.getExtension(), b.getClaw(),b.getIntake(), b.getExtension().getSetpointCM()-200,false),
                new WaitCommand(600),
                new IntakeOutCommand(b.getIntake()),
                new WaitUntilCommand(()->!b.opertator.getButton(GamepadKeys.Button.Y)),
                new IntakeStopCommand(b.getIntake())
                //v2
                //new SetClawPIDCommand(claw, ClawPID.ServoPositions.placing),
                //new SetExtensionCommand(extension, claw, extension.getSetpointCM()-600),
                //new WaitCommand(600),
                //new IntakeOutCommand(intake)
        );
    }

}
