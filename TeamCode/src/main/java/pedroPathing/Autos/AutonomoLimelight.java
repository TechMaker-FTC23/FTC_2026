package pedroPathing.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous
public class AutonomoLimelight extends LinearOpMode {

    private Telemetry telemetryA;
    private Follower follower;
    private Limelight3A limelight;
    private IMU imu;

    // Suas Poses Originais (em CM)
    private final Pose START_POSE = new Pose(0, 0, Math.toRadians(0));
    private final Pose entrega = new Pose(20, 30, Math.toRadians(135));
    private final Pose ColetaMeio = new Pose(80, -75, Math.toRadians(180));
    private final Pose Coletacima = new Pose(160, -5, Math.toRadians(270));

    @Override
    public void runOpMode() throws InterruptedException {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(logoDirection, usbDirection)));
        imu.resetYaw();

        limelight.pipelineSwitch(0);
        limelight.start();

        follower.setStartingPose(START_POSE);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addData("Status", "Autônomo com Fusão de Sensores Inicializado.");
        telemetryA.update();

        waitForStart();

        if (opModeIsActive() &&!isStopRequested()) {
            // --- CICLO 1: START -> COLETA MEIO -> ENTREGA ---

            executePathToPose(ColetaMeio, false, null);
            sleep(500); // Simula ação de coleta

            executePathToPose(entrega, false, null);
            sleep(500); // Simula ação de pontuação

            // --- CICLO 2: ENTREGA -> COLETA MEIO -> ENTREGA ---

            executePathToPose(ColetaMeio, false, null);
            sleep(500);

            executePathToPose(entrega, false, null);
            sleep(500);

            // --- CICLO 3: ENTREGA -> COLETA CIMA -> ENTREGA ---

            executePathToPose(Coletacima, false, null);
            sleep(500);

            executePathToPose(entrega, false, null);
            sleep(500);

            telemetryA.addData("Status", "Autônomo Concluído!");
            telemetryA.update();
        }
    }

    private void executePathToPose(Pose targetPose, boolean isCurve, Pose controlPoint) {
        if (!opModeIsActive()) return;
        PathChain path = follower.pathBuilder()
                .addPath(isCurve? new BezierCurve(new Point(follower.getPose()), new Point(controlPoint), new Point(targetPose))
                        : new BezierLine(new Point(follower.getPose()), new Point(targetPose)))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), targetPose.getHeading())
                .build();
        follower.followPath(path, true);
        waitForPathToFinish();
    }

    private void waitForPathToFinish() {
        while (opModeIsActive() &&!isStopRequested() && follower.isBusy()) {


            correctPoseWithVision();
            follower.update();
            follower.telemetryDebug(telemetryA);
            telemetryA.update();
            idle();
        }
    }
    private void correctPoseWithVision() {
        // Envia a orientação da IMU para a Limelight para obter a melhor pose (MegaTag2)
        limelight.updateRobotOrientation(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        LLResult result = limelight.getLatestResult();

        // Verificação de Confiança
        if (result == null ||!result.isValid()
                | result.getStaleness() > 200) {
            return;
        }

        Pose3D visionPose3D = result.getBotpose_MT2();

        if (visionPose3D!= null) {
            // Converte a Pose3D da Limelight (metros) para a Pose 2D do Pedro Pathing (CM)
            Pose visionPose = new Pose(
                    visionPose3D.getPosition().x * 100.0,
                    visionPose3D.getPosition().y * 100.0,
                    visionPose3D.getOrientation().getYaw(AngleUnit.RADIANS)
            );

            follower.setPose(visionPose);

            telemetryA.addData("Correção de Pose (Visão)", visionPose.toString());
        }
    }
}