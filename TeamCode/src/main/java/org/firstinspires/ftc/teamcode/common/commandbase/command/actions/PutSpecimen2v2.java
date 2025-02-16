package org.firstinspires.ftc.teamcode.common.commandbase.command.actions;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.Bot;
import org.firstinspires.ftc.teamcode.common.commandbase.command.claw.SetClawPIDCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.extension.SetExtensionCommand;

public class PutSpecimen2v2 extends SequentialCommandGroup {
    public PutSpecimen2v2(Bot b){
        addCommands(
                new SetExtensionCommand(b.getExtension(),b.getClaw(),b.getIntake() ,1800,true)

                //new IntakeOutCommand(b.getIntake()),
                //new WaitUntilCommand(()->!b.opertator.getButton(GamepadKeys.Button.Y)),
                //new IntakeStopCommand(b.getIntake())
        );
    }

}
