package org.firstinspires.ftc.teamcode.common.commandbase.command.actions;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.common.Bot;
import org.firstinspires.ftc.teamcode.common.commandbase.command.claw.SetClawPIDCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.intake.IntakeCustomCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.intake.IntakeInCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.pivot.SetPivotAngleCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.ClawPID;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Pivot;

public class EndIntakeCommand extends SequentialCommandGroup {
    public EndIntakeCommand(Bot b){
        addCommands(
                new SetClawPIDCommand(b.getClaw(), ClawPID.ServoPositions.intaking),
                new IntakeInCommand(b.getIntake()),
                new SetPivotAngleCommand(b.getPivot(), b.getClaw(), Pivot.setpoint_intaking,true),

                //new InstantCommand(()->{
                //    pivot.pivotController.setF(0);
                //    pivot.overrideF = true;
                //}),
                new WaitUntilCommand(()->!b.opertator.getButton(GamepadKeys.Button.Y)),
                new SetClawPIDCommand(b.getClaw(), ClawPID.ServoPositions.preintaking),
                //new InstantCommand(()->{
                //    pivot.overrideF = false;
                //}),
                //new SetPivotAngleCommand(pivot, claw, Pivot.setpoint_horizontal),
                new IntakeCustomCommand(b.getIntake(),0.2)
                ///new SetPivotAngleCommand(pivot, claw, Pivot.setpoint_horizontal,true)
                //new SetClawPIDCommand(claw,ClawPID.ServoPositions.safeE)
        );
    }

}
