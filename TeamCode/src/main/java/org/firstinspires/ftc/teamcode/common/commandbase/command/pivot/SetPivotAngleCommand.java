package org.firstinspires.ftc.teamcode.common.commandbase.command.pivot;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.Config;
import org.firstinspires.ftc.teamcode.common.commandbase.command.claw.SetClawPIDCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.ClawPID;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Pivot;

public class SetPivotAngleCommand extends CommandBase {

    private final Pivot pivot;
    private final double angleDeg;
    private final boolean ignoreSafety;

    private final ClawPID claw;

    public SetPivotAngleCommand(Pivot pivot, ClawPID claw, double angleDeg) {
        this(pivot, claw, angleDeg, false);
    }

    public SetPivotAngleCommand(Pivot pivot, ClawPID claw, double angleDeg, boolean ignoreSafety) {
        this.pivot = pivot;
        this.angleDeg = angleDeg;
        this.ignoreSafety = ignoreSafety;
        this.claw = claw;
        //addRequirements(pivot);
    }

    @Override
    public void initialize() {
        new SequentialCommandGroup(
                new ConditionalCommand(
                        new ConditionalCommand(
                                new WaitCommand(0),
                                new SetClawPIDCommand(claw, ClawPID.ServoPositions.safeP),
                                ()->ignoreSafety
                        ),
                        new WaitCommand(0),
                        ()->Math.abs(pivot.getPositionDEG())-Math.abs(angleDeg)>30
                ),
                new InstantCommand(()->{
                    if (ignoreSafety) {
                        pivot.setSetpointIGNORE(angleDeg);
                    }else{
                        pivot.setSetpointDEG(angleDeg);
                    }
                })
        ).schedule();

    }

    @Override
    public boolean isFinished() {
        return pivot.inTolerance();
    }
}
