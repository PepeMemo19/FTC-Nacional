package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.commandbase.command.drive.FollowPathCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.ClawPID;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Extension;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Intake;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Pivot;


import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@Autonomous
public class SampleAuto extends LinearOpMode {

    public static Pose startingPose = new Pose(10.2, 105, 0);

    public static Pose startingPoseIntermediate = new Pose(34, 105, 0);


    public static Pose score1 = new Pose( 12.551029235359252, 137.34585499200296, Math.toRadians(315));

    public static Pose score2 = new Pose(20, 133.1034679809761, Math.toRadians(345.599123));

    public static Pose score3 = new Pose(20, 133.1034679809761, Math.toRadians(0));

    public static Pose score4 = new Pose(20, 133.1034679809761, Math.toRadians(13));



    @Override
    public void runOpMode() {

        Telemetry telem = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        VoltageSensor vs = hardwareMap.voltageSensor.iterator().next();

        Constants.setConstants(FConstants.class, LConstants.class);
        Follower f = new Follower(hardwareMap);

        f.setPose(startingPose);
        f.setMaxPower(0.75);


        SequentialCommandGroup auto = new SequentialCommandGroup(
                //Poner el precargado
                new ParallelCommandGroup(
                        new FollowPathCommand(f, f.pathBuilder()
                                .addPath(
                                        new BezierLine(
                                                new Point(startingPose),
                                                new Point(startingPoseIntermediate)
                                        )
                                )
                                .setLinearHeadingInterpolation(startingPose.getHeading(), startingPoseIntermediate.getHeading())
                                .build()
                        )

                ),
                new WaitCommand(1000),

                new ParallelCommandGroup(
                        new FollowPathCommand(f, f.pathBuilder()
                                .addPath(
                                        new BezierLine(
                                                new Point(startingPoseIntermediate),
                                                new Point(score1)
                                        )
                                )
                                .setLinearHeadingInterpolation(
                                        startingPoseIntermediate.getHeading(), score1.getHeading())
                                .build()
                        )

                ),


                new WaitCommand(1000),

                new ParallelCommandGroup(
                        new FollowPathCommand(f, f.pathBuilder()
                                .addPath(
                                        new BezierLine(
                                                new Point(score1),
                                                new Point(score2)
                                        )
                                )
                                .setLinearHeadingInterpolation(score1.getHeading(), score2.getHeading())
                                .build()
                        )

        ),
        new WaitCommand(1000),

                new ParallelCommandGroup(
                        new FollowPathCommand(f, f.pathBuilder()
                                .addPath(
                                        new BezierLine(
                                                new Point(score2),
                                                new Point(score1)
                                        )
                                )
                                .setLinearHeadingInterpolation(
                                        score2.getHeading(), score1.getHeading())
                                .build()
                        )

                ),
                new WaitCommand(1000),

                new ParallelCommandGroup(
                        new FollowPathCommand(f, f.pathBuilder()
                                .addPath(
                                        new BezierLine(
                                                new Point(score1),
                                                new Point(score3)
                                        )
                                )
                                .setLinearHeadingInterpolation(
                                        score1.getHeading(), score3.getHeading())
                                .build()
                        )

        ),
                new WaitCommand(1000),

                new ParallelCommandGroup(
                        new FollowPathCommand(f, f.pathBuilder()
                                .addPath(
                                        new BezierLine(
                                                new Point(score3),
                                                new Point(score1)
                                        )
                                )
                                .setLinearHeadingInterpolation(
                                        score3.getHeading(), score1.getHeading())
                                .build()
                        )


        ),
                new WaitCommand(1000),

                new ParallelCommandGroup(
                        new FollowPathCommand(f, f.pathBuilder()
                                .addPath(
                                        new BezierLine(
                                                new Point(score1),
                                                new Point(score4)
                                        )
                                )
                                .setLinearHeadingInterpolation(
                                        score1.getHeading(), score4.getHeading())
                                .build()
                        )


        ),
                new WaitCommand(1000),

                new ParallelCommandGroup(
                        new FollowPathCommand(f, f.pathBuilder()
                                .addPath(
                                        new BezierLine(
                                                new Point(score4),
                                                new Point(score1)
                                        )
                                )
                                .setLinearHeadingInterpolation(
                                        score4.getHeading(), score1.getHeading())
                                .build()
                        )
                ),
                new WaitCommand(1000),

                new ParallelCommandGroup(
                        new FollowPathCommand(f, f.pathBuilder()
                                .addPath(
                                        new BezierLine(
                                                new Point(score1),
                                                new Point(score3)
                                        )
                                )
                                .setLinearHeadingInterpolation(
                                        score1.getHeading(), score3.getHeading())
                                .build()
                        )
                )
        );


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
