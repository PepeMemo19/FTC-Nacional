package org.firstinspires.ftc.teamcode.common.commandbase.command.actions;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.Bot;
import org.firstinspires.ftc.teamcode.common.commandbase.command.claw.SetClawPIDCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.extension.SetExtensionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.intake.IntakeOutCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.pivot.SetPivotAngleCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.ClawPID;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Extension;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Pivot;

public class PutSpecimen3 extends SequentialCommandGroup {
    public PutSpecimen3(Bot b){
        addCommands(

                new SetPivotAngleCommand(b.getPivot(),b.getClaw(),Pivot.setpoint_sampling),
                new SetExtensionCommand(b.getExtension(), b.getClaw(),b.getIntake(), Extension.intakeMaxExtension-500,true),
                new SetClawPIDCommand(b.getClaw(), ClawPID.ServoPositions.placing),
                new SetExtensionCommand(b.getExtension(), b.getClaw(),b.getIntake(), Extension.intakeMaxExtension,true),
                new WaitCommand(2000),
                new IntakeOutCommand(b.getIntake()),
                new SetExtensionCommand(b.getExtension(), b.getClaw(),b.getIntake(), 0,true)
                //new WaitUntilCommand(()->!b.opertator.getButton(GamepadKeys.Button.Y)),
                //v2
                //new SetClawPIDCommand(claw, ClawPID.ServoPositions.placing),
                //new SetExtensionCommand(extension, claw, extension.getSetpointCM()-600),
                //new WaitCommand(600),
                //new IntakeOutCommand(intake)
        );
    }

}
