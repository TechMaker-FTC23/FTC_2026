package techmaker;

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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import techmaker.constants.FConstants;
import techmaker.constants.LConstants;
import techmaker.subsystems.ClawSubsystem;
import techmaker.subsystems.ElevatorSubsystem;
import techmaker.subsystems.IntakeSubsystem;
import techmaker.RobotStateManager;
@Disabled
@TeleOp(name = "TeleOp Posicoes Fixas + Mecanismos")
public class TeleOpAll extends OpMode {
    private Follower follower;
    private IMU imu;
    private PathChain pathChain;
    private boolean wasFollowing = false;

    private ClawSubsystem claw;
    private IntakeSubsystem intake;
    private ElevatorSubsystem elevator;
    private RobotStateManager stateManager;

    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose coletaMeio = new Pose(80, -75, Math.toRadians(180));
    private final Pose entrega = new Pose(20, 30, Math.toRadians(135));
    private final Pose começo = new Pose(2, 0, Math.toRadians(0));
    private final Pose voltaEntrega = new Pose(40, 20, Math.toRadians(135));
    private final Pose coletaAlta = new Pose(160, -5, Math.toRadians(270));

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        imu.resetYaw();

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        claw = new ClawSubsystem(hardwareMap,telemetry);
        intake = new IntakeSubsystem(hardwareMap, false,telemetry);
        elevator = new ElevatorSubsystem(hardwareMap,telemetry);

        stateManager = new RobotStateManager(intake, claw);

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
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;

            double rotatedX = x * Math.cos(-heading) - y * Math.sin(-heading);
            double rotatedY = x * Math.sin(-heading) + y * Math.cos(-heading);

            follower.setTeleOpMovementVectors(rotatedY, rotatedX, turn, false);

            if (gamepad1.dpad_down) imu.resetYaw();

            if (gamepad1.square) moveTo(coletaMeio);
            else if (gamepad1.cross) moveTo(começo);
            else if (gamepad1.y) moveTo(entrega);
            else if (gamepad1.dpad_up) curvaBezier(entrega, voltaEntrega, começo, Math.toRadians(0));
            else if (gamepad1.circle) curvaBezier(new Pose(60, 30, 0), new Pose(150, 30, 0), coletaAlta, Math.toRadians(90));
        } else {
            wasFollowing = true;
        }

        // Atualiza mecanismos com gamepad2
        stateManager.updateIntake(gamepad2);
        stateManager.updateClaw(gamepad2);
        stateManager.runAutoCycle(gamepad2);

        follower.telemetryDebug(telemetry);
    }

    private void moveTo(Pose destino) {
        pathChain = follower.pathBuilder()
                .addPath(new BezierLine(new Point(follower.poseUpdater.getPose()), new Point(destino)))
                .setLinearHeadingInterpolation(follower.poseUpdater.getPose().getHeading(), destino.getHeading())
                .build();
        follower.followPath(pathChain, true);
    }

    private void curvaBezier(Pose p0, Pose p1, Pose p2, double heading) {
        pathChain = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(p0), new Point(p1), new Point(p2)))
                .setConstantHeadingInterpolation(heading)
                .build();
        follower.followPath(pathChain, true);
    }
}
