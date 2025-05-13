package pedroPathing.examples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * This is the Circle autonomous OpMode. It runs the robot in a PathChain that's actually not quite
 * a circle, but some Bezier curves that have control points set essentially in a square. However,
 * it turns enough to tune your centripetal force correction and some of your heading. Some lag in
 * heading is to be expected.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */

@Autonomous (name = "PathTest", group = "Examples")
public class PathTest extends OpMode {
    private Telemetry telemetryA;
    private Path line;
    private Follower follower;
    int path = 1;
    private final Pose startPose = new Pose(0, 0, Math.toRadians(90));
    private final Pose secondPose = new Pose(100,0, Math.toRadians(90));
    private final Pose thirdPose = new Pose(100,-75, Math.toRadians(0));
    private final Pose stopPose = new Pose(0, -75, Math.toRadians(0));
       /**
     * This initializes the Follower and creates the PathChain for the "circle". Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        line = new Path(new BezierLine(new Point(startPose), new Point(secondPose)));
        line.setLinearHeadingInterpolation(startPose.getHeading(), secondPose.getHeading());


        follower.followPath(line);


        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.update();
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {
        follower.update();
        if (!follower.isBusy()) {
            path++;
            /*if(path == 2){
                line = new Path(new BezierLine(new Point(secondPose), new Point(thirdPose)));
                line.setLinearHeadingInterpolation(secondPose.getHeading(), thirdPose.getHeading());


                follower.followPath(line);
            }

            else{*/
                follower.holdPoint(secondPose);
            //}


        }

        follower.telemetryDebug(telemetryA);
    }
}
