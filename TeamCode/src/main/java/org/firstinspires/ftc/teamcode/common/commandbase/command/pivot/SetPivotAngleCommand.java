package org.firstinspires.ftc.teamcode.common.commandbase.command.pivot;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.Config;
import org.firstinspires.ftc.teamcode.common.commandbase.command.claw.SetClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.ClawServo;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Pivot;

public class SetPivotAngleCommand extends CommandBase {

    private final Pivot pivot;
    private final double angleDeg;
    private final boolean ignoreSafety;

    private final ClawServo claw;

    public SetPivotAngleCommand(Pivot pivot, ClawServo claw, double angleDeg) {
        this(pivot, claw, angleDeg, false);
    }

    public SetPivotAngleCommand(Pivot pivot, ClawServo claw, double angleDeg, boolean ignoreSafety) {
        this.pivot = pivot;
        this.angleDeg = angleDeg;
        this.ignoreSafety = ignoreSafety;
        this.claw = claw;
        addRequirements(pivot);
    }

    @Override
    public void initialize() {
        new ConditionalCommand(
                new SequentialCommandGroup(
                        new SetClawCommand(claw,ClawServo.ServoPositions.safe),
                        new WaitCommand(600),
                        new InstantCommand(()->{
                            if (ignoreSafety) {
                                pivot.setSetpointIGNORE(angleDeg);
                            }else{
                                pivot.setSetpointDEG(angleDeg);
                            }
                        })
                ),
                new InstantCommand(()->{
                    if (ignoreSafety) {
                        pivot.setSetpointIGNORE(angleDeg);
                    }else {
                        pivot.setSetpointDEG(angleDeg);
                    }
                }),
                ()->claw.clawPivot.getPosition()!=ClawServo.ServoPositions.safe
        ).schedule();

    }

    @Override
    public boolean isFinished() {
        return (Math.abs(pivot.getPositionDEG() - angleDeg) <= Config.pivot_tolerance);
    }
}
