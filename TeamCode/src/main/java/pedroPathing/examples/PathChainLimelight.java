package pedroPathing.examples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Drawing;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
@TeleOp
public class PathChainLimelight extends OpMode {
    private Limelight3A limelight;
    public static double multiplier = 100.0; // converte metros para centímetros
    public static AngleUnit angle = AngleUnit.DEGREES;
    public static boolean isPedroPathing = true;
    public static double PositionX = 0;
    public static double PositionY = 0;

    private Telemetry telemetryA;
    private PathChain pathChain;
    private Follower follower;
    private boolean wasFollowing = false;

    // Definição de poses
    private final Pose startPose   = new Pose(0, 0, Math.toRadians(0));
    private final Pose StarPose    = new Pose(1, 0, Math.toRadians(0));
    private final Pose Começo      = new Pose(2, 0, Math.toRadians(0));
    private final Pose ColetaMeio  = new Pose(80, -75, Math.toRadians(180));
    private final Pose entrega     = new Pose(20, 30, Math.toRadians(135));
    private final Pose voltaEntrega= new Pose(40, 20, Math.toRadians(135));
    private final Pose ColetaCima  = new Pose(160, -5, Math.toRadians(270));

    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

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

            // Controle manual
            follower.setTeleOpMovementVectors(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    false);

            // Seleção de trajetórias via gamepad
            if (gamepad1.square) {
                pathChain = follower.pathBuilder()
                        .addPath(new BezierLine(
                                new Point(follower.poseUpdater.getPose()),
                                new Point(ColetaMeio)))
                        .setLinearHeadingInterpolation(
                                follower.poseUpdater.getPose().getHeading(),
                                ColetaMeio.getHeading())
                        .build();
                follower.followPath(pathChain, true);
            } else if (gamepad1.cross) {
                pathChain = follower.pathBuilder()
                        .addPath(new BezierLine(
                                new Point(follower.poseUpdater.getPose()),
                                new Point(Começo)))
                        .setLinearHeadingInterpolation(
                                follower.poseUpdater.getPose().getHeading(),
                                Começo.getHeading())
                        .build();
                follower.followPath(pathChain, true);
            } else if (gamepad1.y) {
                pathChain = follower.pathBuilder()
                        .addPath(new BezierLine(
                                new Point(follower.poseUpdater.getPose()),
                                new Point(entrega)))
                        .setLinearHeadingInterpolation(
                                follower.poseUpdater.getPose().getHeading(),
                                entrega.getHeading())
                        .build();
                follower.followPath(pathChain, true);
            } else if (gamepad1.dpad_up) {
                pathChain = follower.pathBuilder()
                        .addPath(new BezierCurve(
                                new Point(entrega),
                                new Point(voltaEntrega),
                                new Point(Começo)))
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .build();
                follower.followPath(pathChain, true);
            } else if (gamepad1.circle) {
                pathChain = follower.pathBuilder()
                        .addPath(new BezierCurve(
                                new Point(60.00, 30.00),
                                new Point(150, 30),
                                new Point(ColetaCima)))
                        .setConstantHeadingInterpolation(Math.toRadians(90))
                        .build();
                follower.followPath(pathChain, true);
            }
        } else {
            wasFollowing = true;
        }

        // ** Atualização de pose via Limelight (AprilTags) **
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botPose = result.getBotpose_MT2();

            double x = botPose.getPosition().x * multiplier;
            double y = botPose.getPosition().y * multiplier;
            double heading = botPose.getOrientation().getYaw(angle);

            // Atualiza pose no poseUpdater
            follower.poseUpdater.setStartingPose(new Pose(x, y, heading));
            // Armazena último pose conhecido
            PositionX = x;
            PositionY = y;

            telemetryA.addData("Limelight Pose", String.format("x: %.2f y: %.2f θ: %.2f", x, y, heading));
            Drawing.drawRobot(new Pose(x, y, heading), "#008000");
        } else {
            // Desenha última pose conhecida em vermelho caso não haja detecção
            Drawing.drawRobot(new Pose(PositionX, PositionY, 0), "#FF0000");
        }

        // Desenha a pose estimada pelo odômetro/integrador
        telemetryA.addData("Pose", follower.getPose().toString());
        Drawing.drawRobot(follower.getPose(), "#000080");
        Drawing.sendPacket();
        telemetryA.update();
    }
}
