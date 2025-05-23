package pedroPathing.examples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp
public class PathChainTest extends OpMode {
    private Telemetry telemetryA;
    private PathChain pathChain;
    private Follower follower;
    int path = 1;
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose StarPose = new Pose(1, 0, Math.toRadians(0));
    private final Pose Começo = new Pose(2, 0, Math.toRadians(0));
    private final Pose Coletacima = new Pose(160, -5, Math.toRadians(270));
    private final Pose voltaEntrga = new Pose(40, 20, Math.toRadians(135));
    private final Pose ColetaMeio = new Pose(80, -75, Math.toRadians(180));
    private final Pose entrega = new Pose(20, 30, Math.toRadians(135));
    private boolean wasFollowing = false;

    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        pathChain = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(StarPose)))
                .build();

        follower.followPath(pathChain, true);
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();

        if (!follower.isBusy()) {
            if (wasFollowing) {
                follower.startTeleopDrive();
                wasFollowing = false;
            }

            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);

            if (gamepad1.square) {
                pathChain = follower.pathBuilder()
                        .addPath(new BezierLine(new Point(follower.poseUpdater.getPose()), new Point(ColetaMeio)))
                        .setLinearHeadingInterpolation(follower.poseUpdater.getPose().getHeading(), ColetaMeio.getHeading())
                        .build();
                follower.followPath(pathChain, true);
            } else if (gamepad1.cross) {
                pathChain = follower.pathBuilder()
                        .addPath(new BezierLine(new Point(follower.poseUpdater.getPose()), new Point(Começo)))
                        .setLinearHeadingInterpolation(follower.poseUpdater.getPose().getHeading(), Começo.getHeading())
                        .build();
                follower.followPath(pathChain, true);
            } else if (gamepad1.y) {
                pathChain = follower.pathBuilder()
                        .addPath(new BezierLine(new Point(follower.poseUpdater.getPose()), new Point(entrega)))
                        .setLinearHeadingInterpolation(follower.poseUpdater.getPose().getHeading(), entrega.getHeading())
                        .build();
                follower.followPath(pathChain, true);
            } else if (gamepad1.dpad_up) {
                pathChain = follower.pathBuilder()
                        .addPath(new BezierCurve(
                                new Point(entrega),
                                new Point(voltaEntrga),
                                new Point(Começo)))
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .build();
                follower.followPath(pathChain, true);
            } else if (gamepad1.circle) {
                pathChain = follower.pathBuilder()
                        .addPath(new BezierCurve(
                                new Point(60.00, 30.00),
                                new Point(150, 30),
                                new Point(Coletacima)))
                        .setConstantHeadingInterpolation(Math.toRadians(90))
                        .build();
                follower.followPath(pathChain, true);
            }
        } else {
            wasFollowing = true;
        }

        follower.telemetryDebug(telemetryA);
    }
}
