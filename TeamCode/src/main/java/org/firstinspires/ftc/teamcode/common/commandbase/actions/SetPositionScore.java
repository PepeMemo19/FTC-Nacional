package org.firstinspires.ftc.teamcode.common.commandbase.actions;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.Bot;
import org.firstinspires.ftc.teamcode.common.commandbase.command.extension.SetExtensionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.pivot.SetPivotAngleCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.ClawServo;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Extension;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Pivot;

@Config
public class SetPositionScore extends CommandBase {
    private Bot bot;
    private Pivot pivot;
    private Extension extension;
    private ClawServo claw;

    private static double HIGH_SPECIMEN_ANGLE = 45, LOW_SPECIMEN_ANGLE = 30;
    private static double HIGH_SPECIMEN_LENGTH = 20, LOW_SPECIMEN_LENGTH = 0;
    private static double SAMPLE_ANGLE = Pivot.setpoint_vertical;
    private static double HIGH_SAMPLE_LENGTH = 20, LOW_SAMPLE_LENGTH = 0;
    public SetPositionScore(Bot bot){
        this.bot = bot;

        pivot = bot.getPivot();
        extension = bot.getExtension();
        claw = bot.getClaw();
    }

    @Override
    public void initialize(){
        new ConditionalCommand(
                new SequentialCommandGroup(
                        new SetPivotAngleCommand(pivot, claw, HIGH_SPECIMEN_ANGLE),
                        new SetExtensionCommand(extension, claw, HIGH_SPECIMEN_LENGTH)
                ),
                new SequentialCommandGroup(
                        new SetPivotAngleCommand(pivot, claw, SAMPLE_ANGLE),
                        new SetExtensionCommand(extension, claw, HIGH_SAMPLE_LENGTH)
                ),
                () -> bot.getMode() == Bot.Modes.SPECIMENS
        );
    }
}
