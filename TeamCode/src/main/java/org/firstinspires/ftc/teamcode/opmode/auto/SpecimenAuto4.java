package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.Bot;
import org.firstinspires.ftc.teamcode.common.commandbase.command.actions.PutSpecimen;
import org.firstinspires.ftc.teamcode.common.commandbase.command.actions.PutSpecimen2v2;
import org.firstinspires.ftc.teamcode.common.commandbase.command.actions.PutSpecimen3;
import org.firstinspires.ftc.teamcode.common.commandbase.command.actions.StartDepositSpecimen;
import org.firstinspires.ftc.teamcode.common.commandbase.command.drive.FollowPathCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.pivot.SetPivotAngleCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Claw;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.ClawPID;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Extension;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Pivot;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


//new InstantCommand(()->{arm.up();}),  convierte código a un comando
//SequentialCommandGroup                los comandos dentro corren en sequencia
//ParallelCommandGroup                  los comandos dentro corren en paralelo
//new WaitCommand(1000),                espera 1000 milisegundos

@Autonomous(name="Specimen Auto4")
//@Autonomous(name="Specimen Auto", preselectTeleOp="TeleOp")
public class SpecimenAuto4 extends LinearOpMode {

    public static Pose startingPose = new Pose(9, 55, 0);
    public static Pose parkPose = new Pose(18, 26, 0);
    public static double scoreX = 30.4;

    public static Pose score0 = new Pose(scoreX, 55, 0);
    public static Pose score1 = new Pose(scoreX, 79, 0);
    public static Pose score2 = new Pose(scoreX, 79, 0);
    public static Pose score3 = new Pose(scoreX, 75, 0);
    public static Pose score4 = new Pose(scoreX, 71, 0);
    public static Pose score5 = new Pose(scoreX, 67, 0);

    public static Pose intake1 = new Pose(17.064935064935064, 40.67532467532467, Math.toRadians(-180));

    public static double push1Y = 27.584415584415588;
    public static double push2Y = 17.06493506493507;
    public static double push3Y = 9.818181818181825;

    public static double downX = 14;
    public static double upX = 57.740259740259745;


    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().reset();

        Telemetry telem = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        VoltageSensor vs = hardwareMap.voltageSensor.iterator().next();

        Constants.setConstants(FConstants.class, LConstants.class);
        Follower f = new Follower(hardwareMap);

        f.setPose(startingPose);
        f.setMaxPower(0.75);

        double pivotVConstant = 13.4/vs.getVoltage();

        Bot bot = new Bot(telem, hardwareMap, new GamepadEx(gamepad1), new GamepadEx(gamepad2), false);

        //inicializar subsistemas aqui
        //Arm arm = new Arm(hardwareMap, telemetry);





        SequentialCommandGroup auto = new SequentialCommandGroup(
                //Poner el precargado
                new ParallelCommandGroup(
                        new FollowPathCommand(f, f.pathBuilder()
                                .addPath(
                                        new BezierLine(
                                                new Point(startingPose),
                                                new Point(score0)
                                        )
                                )
                                .setConstantHeadingInterpolation(score1.getHeading())
                                .build()
                        )
                ),
                new WaitCommand(3000),
                new PutSpecimen3(bot),
                new WaitCommand(3000),

                new WaitCommand(3000)

        );

        // Wait for start and schedule auto command group
        waitForStart();
        CommandScheduler.getInstance().schedule(auto);

        Extension extension = bot.getExtension();
        ClawPID claw = bot.getClaw();
        Pivot pivot = bot.getPivot();

        // Opmode loop
        while (opModeIsActive()) {
            f.setMaxPower(10.0 / vs.getVoltage());
            CommandScheduler.getInstance().run();
            f.update();
            f.telemetryDebug(telem);
            claw.periodic();
            extension.periodic();
            pivot.periodic();
        }
    }
}