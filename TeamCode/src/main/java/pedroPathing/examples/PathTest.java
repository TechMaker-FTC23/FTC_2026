package pedroPathing.examples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.BezierPoint;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

@TeleOp(name = "PathTest", group = "Examples")
public class PathTest extends OpMode {
    private Telemetry telemetryA;
    private Path line;
    private Follower follower;
    int path = 1;
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose StarPose = new Pose(1, 0, Math.toRadians(0));
    private final Pose Começo = new Pose(2, 0, Math.toRadians(0));
    private final Pose Coletacima = new Pose(160, -5, Math.toRadians(270));
    private final Pose voltaEntrga = new Pose(40,20, Math.toRadians(135));
    private final Pose ColetaMeio = new Pose(80,-75, Math.toRadians(180));
    private final Pose entrega = new Pose(20,30, Math.toRadians(135));
    private boolean wasFollowing = false;
       /**
     * This initializes the Follower and creates the PathChain for the "circle". Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        line = new Path(new BezierPoint(StarPose));
        follower.followPath(line);
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.update();
    }
    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        follower.startTeleopDrive();
    }
    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {
        follower.update();
        if (!follower.isBusy()) {
            if(wasFollowing){
                follower.startTeleopDrive();
                wasFollowing = false;
            }
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
            if(gamepad1.square){
                line = new Path(new BezierLine(new Point(follower.poseUpdater.getPose()), new Point(ColetaMeio)));
                line.setLinearHeadingInterpolation(follower.poseUpdater.getPose().getHeading(), ColetaMeio.getHeading());


                follower.followPath(line);
            }
            else if(gamepad1.cross){
                line = new Path(new BezierLine(new Point(follower.poseUpdater.getPose()), new Point(Começo)));
                line.setLinearHeadingInterpolation(follower.poseUpdater.getPose().getHeading(), Começo.getHeading());


                follower.followPath(line);
            }
            else if(gamepad1.y){
                line = new Path(new BezierLine(new Point(follower.poseUpdater.getPose()), new Point(entrega)));
                line.setLinearHeadingInterpolation(follower.poseUpdater.getPose().getHeading(), entrega.getHeading());


                follower.followPath(line);
            }
            else if(gamepad1.dpad_up) {
                line = new Path(new BezierCurve(
                        new Point(entrega),
                        new Point(voltaEntrga),
                        new Point(Começo)));

                line.setConstantHeadingInterpolation(Math.toRadians(0));


                follower.followPath(line);
            }
                 else if(gamepad1.circle){
                    line = new Path(new BezierCurve(
                            new Point(60.00, 30.),
                            new Point(150, 30),
                            new Point(Coletacima)));

                    line.setConstantHeadingInterpolation(Math.toRadians(90));

                    follower.followPath(line);
            }
        }
        else{
            wasFollowing = true;
        }

        follower.telemetryDebug(telemetryA);
    }
}
