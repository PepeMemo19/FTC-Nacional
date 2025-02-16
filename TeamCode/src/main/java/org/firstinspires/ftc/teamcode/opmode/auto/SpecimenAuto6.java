package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
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
import org.firstinspires.ftc.teamcode.common.commandbase.command.drive.FollowPathCommand;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


//new InstantCommand(()->{arm.up();}),  convierte código a un comando
//SequentialCommandGroup                los comandos dentro corren en sequencia
//ParallelCommandGroup                  los comandos dentro corren en paralelo
//new WaitCommand(1000),                espera 1000 milisegundos

@Autonomous(name="Specimen Auto6")
//@Autonomous(name="Specimen Auto", preselectTeleOp="TeleOp")
public class SpecimenAuto6 extends LinearOpMode {

    public static Pose startingPose = new Pose(9, 55, 0);
    public static Pose parkPose = new Pose(18, 26, 0);

    public static double scoreX = 37;
    public static Pose score1 = new Pose(scoreX, 83, 0);
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

        //inicializar subsistemas aqui
        //Arm arm = new Arm(hardwareMap, telemetry);

        SequentialCommandGroup auto = new SequentialCommandGroup(
                //Poner el precargado
                new ParallelCommandGroup(
                        new FollowPathCommand(f, f.pathBuilder()
                                .addPath(
                                        new BezierLine(
                                                new Point(startingPose),
                                                new Point(score1)
                                        )
                                )
                                .setConstantHeadingInterpolation(score1.getHeading())
                                .build()
                        )
                ),
                //arrastrar cubos
                new ParallelCommandGroup(
                        new FollowPathCommand(f, f.pathBuilder()
                                .addPath(
                                        new BezierCurve(
                                                new Point(new Pose(18.467532467532468, 39.506493506493506, Math.toRadians(0))),//preparar
                                                new Point(new Pose(upX, 34.36363636363637, Math.toRadians(0)))//subir
                                                //new Point(new Pose(64.05194805194806, 9.818181818181825, Math.toRadians(0)))
                                        )
                                )
                                .setConstantHeadingInterpolation(score1.getHeading())
                                //sample 1
                                .addPath(
                                        new BezierLine(
                                                new Point(new Pose(upX, 34.36363636363637, Math.toRadians(0))),
                                                new Point(new Pose(upX, push1Y, Math.toRadians(0)))//lateral 1
                                        )
                                )
                                .setConstantHeadingInterpolation(score1.getHeading())
                                .addPath(
                                        new BezierLine(
                                                new Point(new Pose(upX, push1Y, Math.toRadians(0))),//lateral 1
                                                new Point(new Pose(downX, push1Y, Math.toRadians(0)))//bajar
                                        )
                                )
                                .setConstantHeadingInterpolation(score1.getHeading())
                                .addPath(
                                        new BezierLine(
                                                new Point(new Pose(downX, push1Y, Math.toRadians(0))),//bajar
                                                new Point(new Pose(upX, push1Y, Math.toRadians(0)))//subir
                                        )
                                )
                                .setConstantHeadingInterpolation(score1.getHeading())
                                //sample 2
                                .addPath(
                                        new BezierLine(
                                                new Point(new Pose(upX, push1Y, Math.toRadians(0))),//subir
                                                new Point(new Pose(upX, push2Y, Math.toRadians(0)))//lateral 2
                                        )
                                )
                                .setConstantHeadingInterpolation(score1.getHeading())
                                .addPath(
                                        new BezierLine(
                                                new Point(new Pose(upX, push2Y, Math.toRadians(0))),//lateral 2
                                                new Point(new Pose(downX, push2Y, Math.toRadians(0)))//bajar
                                        )
                                )
                                .setConstantHeadingInterpolation(score1.getHeading())
                                /*
                                .addPath(
                                        new BezierLine(
                                                new Point(new Pose(downX, push2Y, Math.toRadians(0))),//bajar
                                                new Point(new Pose(upX, push2Y, Math.toRadians(0)))//subir
                                        )
                                )
                                .setConstantHeadingInterpolation(score1.getHeading())

                                //sample 3

                                .addPath(
                                        new BezierLine(
                                                new Point(new Pose(upX, push2Y, Math.toRadians(0))),//subir
                                                new Point(new Pose(upX, push3Y, Math.toRadians(0)))//lateral 3
                                        )
                                )
                                .setConstantHeadingInterpolation(score1.getHeading())
                                .addPath(
                                        new BezierLine(
                                                new Point(new Pose(upX, push3Y, Math.toRadians(0))),//lateral 3
                                                new Point(new Pose(downX, push3Y, Math.toRadians(0)))//bajar
                                        )
                                )
                                .setConstantHeadingInterpolation(score1.getHeading())*/
                                //.setLinearHeadingInterpolation(score1.getHeading(), intake1.getHeading())
                                .build()
                        )
                ),

                new WaitCommand(15000000),

                new WaitCommand(500),
                new WaitCommand(150),


                new FollowPathCommand(f, f.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Point(new Pose(downX, push3Y, Math.toRadians(0))),//bajar
                                        new Point(intake1),
                                        new Point(score2)
                                )
                        )
                        .setConstantHeadingInterpolation(score2.getHeading())
                        .build()
                ),

                new FollowPathCommand(f, f.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(intake1),
                                        new Point(score3)
                                )
                        )
                        .setConstantHeadingInterpolation(score3.getHeading())
                        .build()
                ),

                new FollowPathCommand(f, f.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(score3),
                                        new Point(intake1)
                                )
                        )
                        .setConstantHeadingInterpolation(intake1.getHeading())
                        .build()
                ),
                new FollowPathCommand(f, f.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(intake1),
                                        new Point(score4)
                                )
                        )
                        .setConstantHeadingInterpolation(score4.getHeading())
                        .build()
                ),
                new FollowPathCommand(f, f.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(score4),
                                        new Point(intake1)
                                )
                        )
                        .setConstantHeadingInterpolation(intake1.getHeading())
                        .build()
                ),
                new FollowPathCommand(f, f.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(intake1),
                                        new Point(score5)
                                )
                        )
                        .setConstantHeadingInterpolation(score5.getHeading())
                        .build()
                ),
                new FollowPathCommand(f, f.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(score5),
                                        new Point(parkPose)
                                )
                        )
                        .setLinearHeadingInterpolation(score5.getHeading(), parkPose.getHeading())
                        .build()
                )
        );

        // Wait for start and schedule auto command group
        waitForStart();
        CommandScheduler.getInstance().schedule(auto);

        // Opmode loop
        while (opModeIsActive()) {
            f.setMaxPower(10.0 / vs.getVoltage());
            CommandScheduler.getInstance().run();
            f.update();
            f.telemetryDebug(telem);
        }
    }
}