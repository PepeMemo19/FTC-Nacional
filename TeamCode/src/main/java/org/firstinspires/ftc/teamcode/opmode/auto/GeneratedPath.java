package org.firstinspires.ftc.teamcode.opmode.auto;

import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.Point;

public class GeneratedPath {
    public PathBuilder builder = new PathBuilder();

    public GeneratedPath() {
        builder = new PathBuilder();

        builder
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(9.757, 84.983, Point.CARTESIAN),
                                new Point(30.000, 84.983, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(30.000, 84.983, Point.CARTESIAN),
                                new Point(30.000, 70.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation();
    }
}