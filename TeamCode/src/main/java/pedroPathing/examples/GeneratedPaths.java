package pedroPathing.examples;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
public class GeneratedPaths {

    public static PathBuilder builder = new PathBuilder();
    public static Pose startPoint = new Pose(8,80,0);
    public static PathChain line1 = builder
            .addPath(
                    new BezierLine(
                            new Point(8.000, 80.000, Point.CARTESIAN),
                            new Point(47.409, 80.197, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();

    public static PathChain line2 = builder
            .addPath(
                    new BezierCurve(
                            new Point(47.409, 80.197, Point.CARTESIAN),
                            new Point(88.394, 100.578, Point.CARTESIAN),
                            new Point(47.188, 119.631, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();

    public static PathChain line3 = builder
            .addPath(
                    new BezierLine(
                            new Point(47.188, 119.631, Point.CARTESIAN),
                            new Point(3.545, 139.126, Point.CARTESIAN)
                    )
            )
            .setTangentHeadingInterpolation()
            .build();
}


