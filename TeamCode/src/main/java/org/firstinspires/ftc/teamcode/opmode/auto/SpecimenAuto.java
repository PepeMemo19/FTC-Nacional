package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Vector2d;
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

@Autonomous(name="Specimen Auto")
//@Autonomous(name="Specimen Auto", preselectTeleOp="TeleOp")
public class SpecimenAuto extends LinearOpMode {

    public static Pose startingPose = new Pose(9, 55, 0);
    public static Pose parkPose = new Pose(18, 26, 0);

    public static Pose score1 = new Pose(38, 83, 0);
    public static Pose score2 = new Pose(38, 79, Math.toRadians(180));
    public static Pose score3 = new Pose(38, 75, Math.toRadians(180));
    public static Pose score4 = new Pose(38, 71, Math.toRadians(180));
    public static Pose score5 = new Pose(38, 67, Math.toRadians(180));

    public static Pose specIntake = new Pose(18, 26, Math.toRadians(180));

    public static Pose intake1 = new Pose(30, 46, Math.toRadians(-58));
    public static Pose intake1Control = new Pose(9, 55);
    public static Pose intake1Shuttle = new Pose(30, 40, Math.toRadians(-140));

    public static Pose intake2 = new Pose(30, 35, Math.toRadians(-60));
    public static Pose intake2Shuttle = new Pose(30, 40, Math.toRadians(-140));

    public static Pose intake3 = new Pose(30, 26, Math.toRadians(-54));
    public static Pose intake3Shuttle = new Pose(19, 26, Math.toRadians(180));
    public static Pose intake3ShuttleControl = new Pose(30, 24);


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





        SequentialCommandGroup auto = new SequentialCommandGroup(
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
                new ParallelCommandGroup(
                        new FollowPathCommand(f, f.pathBuilder()
                                .addPath(
                                        new BezierCurve(
                                                new Point(score1),
                                                new Point(intake1Control),
                                                new Point(intake1)
                                        )
                                )
                                .setLinearHeadingInterpolation(score1.getHeading(), intake1.getHeading())
                                .build()
                        )
                ),
                new FollowPathCommand(f, f.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(intake1),
                                        new Point(intake1Shuttle)
                                )
                        )
                        .setLinearHeadingInterpolation(intake1.getHeading(), intake1Shuttle.getHeading())
                        .build()
                ),
                new WaitCommand(150),
                new WaitCommand(150),
                new FollowPathCommand(f, f.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(intake1Shuttle),
                                        new Point(intake2)
                                )
                        )
                        .setLinearHeadingInterpolation(intake1Shuttle.getHeading(), intake2.getHeading())
                        .build()
                ),
                new WaitCommand(300),
                new WaitCommand(150),
                new FollowPathCommand(f, f.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(intake2),
                                        new Point(intake2Shuttle)
                                )
                        )
                        .setLinearHeadingInterpolation(intake2.getHeading(), intake2Shuttle.getHeading())
                        .build()
                ),
                new WaitCommand(150),
                new WaitCommand(150),
                new FollowPathCommand(f, f.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(intake2Shuttle),
                                        new Point(intake3)
                                )
                        )
                        .setLinearHeadingInterpolation(intake2Shuttle.getHeading(), intake3.getHeading(), 0.5)
                        .build()
                ),
                new WaitCommand(500),
                new WaitCommand(150),
                new FollowPathCommand(f, f.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Point(intake3),
                                        new Point(intake3ShuttleControl),
                                        new Point(intake3Shuttle)
                                )
                        )
                        .setLinearHeadingInterpolation(intake3.getHeading(), intake3Shuttle.getHeading(), 0.6)
                        .build()
                ),

                new FollowPathCommand(f, f.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(intake3Shuttle),
                                        new Point(score2)
                                )
                        )
                        .setConstantHeadingInterpolation(score2.getHeading())
                        .build()
                ),

                new FollowPathCommand(f, f.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(specIntake),
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
                                        new Point(specIntake)
                                )
                        )
                        .setConstantHeadingInterpolation(specIntake.getHeading())
                        .build()
                ),
                new FollowPathCommand(f, f.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(specIntake),
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
                                        new Point(specIntake)
                                )
                        )
                        .setConstantHeadingInterpolation(specIntake.getHeading())
                        .build()
                ),
                new FollowPathCommand(f, f.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(specIntake),
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