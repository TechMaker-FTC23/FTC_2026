package techmaker.examples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.BezierPoint;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import techmaker.constants.FConstants;
import techmaker.constants.LConstants;


@TeleOp
public class PathChainTest extends OpMode {
    private Telemetry telemetryA;
    private PathChain pathChain;
    private Follower follower;
    private IMU imu;
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose Coletacima = new Pose(95, -80, Math.toRadians(0));
    private final Pose voltaEntrga = new Pose(40, -20, Math.toRadians(135));
    private final Pose ColetaMeio = new Pose(100, -30, Math.toRadians(-90));
    private final Pose entrega = new Pose(-20, -20, Math.toRadians(135));
    private boolean wasFollowing = false;

    @Override
    public void init() {

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        pathChain = follower.pathBuilder()
                .addPath(new BezierPoint(new Point(startPose)))
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

             if (gamepad1.dpad_right) {
                follower.setPose(startPose);
                pathChain = follower.pathBuilder()
                        .addPath(new BezierLine(
                                new Point(startPose),
                                new Point(10, 10)))
                        .build();
                follower.followPath(pathChain, true);
            }

        }

        follower.telemetryDebug(telemetryA);

    }
}
