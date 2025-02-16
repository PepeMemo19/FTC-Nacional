package org.firstinspires.ftc.teamcode.common.commandbase.command.actions;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.Bot;
import org.firstinspires.ftc.teamcode.common.commandbase.command.claw.SetClawPIDCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.extension.SetExtensionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.pivot.SetPivotAngleCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Pivot;

public class StartDepositSpecimen2v2 extends SequentialCommandGroup {
    public StartDepositSpecimen2v2(Bot b){
        addCommands(
                new SetPivotAngleCommand(b.getPivot(), b.getClaw(), Pivot.setpoint_vertical),
                new SetExtensionCommand(b.getExtension(), b.getClaw(), b.getIntake(), 1100,true),
                new SetClawPIDCommand(b.getClaw(), 66)





        );
    }

}
