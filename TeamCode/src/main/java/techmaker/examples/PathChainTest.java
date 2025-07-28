package techmaker.examples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import techmaker.constants.FConstants;
import techmaker.constants.LConstants;
@Disabled

@TeleOp
public class PathChainTest extends OpMode {
    private Telemetry telemetryA;
    private PathChain pathChain;
    private Follower follower;
    private IMU imu;
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

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

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


            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double y_stick = -gamepad1.left_stick_y;
            double x_stick = gamepad1.left_stick_x;
            double turn_stick = -gamepad1.right_stick_x;

            double rotatedX = x_stick * Math.cos(-heading) - y_stick * Math.sin(-heading);
            double rotatedY = x_stick * Math.sin(-heading) + y_stick * Math.cos(-heading);

            follower.setTeleOpMovementVectors(rotatedY, rotatedX, turn_stick, false);

            if (gamepad1.dpad_down) {
                imu.resetYaw();
            }

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