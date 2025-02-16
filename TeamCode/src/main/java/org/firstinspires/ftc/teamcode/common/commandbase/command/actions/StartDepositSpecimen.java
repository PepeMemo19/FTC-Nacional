package org.firstinspires.ftc.teamcode.common.commandbase.command.actions;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.Bot;
import org.firstinspires.ftc.teamcode.common.commandbase.command.claw.SetClawPIDCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.extension.SetExtensionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.pivot.SetPivotAngleCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.ClawPID;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Extension;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Pivot;

public class StartDepositSpecimen extends SequentialCommandGroup {
    public StartDepositSpecimen(Bot b){
        addCommands(
                new SetPivotAngleCommand(b.getPivot(), b.getClaw(), Pivot.setpoint_vertical),
                //new SetExtensionCommand(extension, claw, Extension.specimening),
                //new SetClawPIDCommand(claw, ClawPID.ServoPositions.specimening)
                new SetExtensionCommand(b.getExtension(), b.getClaw(), b.getIntake(), Extension.specimening,false),
                new SetClawPIDCommand(b.getClaw(), ClawPID.ServoPositions.specimening)
        );
    }

}
