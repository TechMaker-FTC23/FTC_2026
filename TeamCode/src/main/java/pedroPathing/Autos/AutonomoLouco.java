package pedroPathing.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult; // Import necessário
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import java.util.Arrays;
import java.util.List;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
@Autonomous
public class AutonomoLouco extends LinearOpMode {

    private Follower follower;
    private Limelight3A limelight;
    private IMU imu;
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    public static List<Integer> TARGET_APRILTAG_IDS = Arrays.asList(11, 12, 13, 14, 15, 16);
    public static double TARGET_DISTANCE_CM = 30.0;
    public static double ALIGNMENT_TOLERANCE_DEGREES = 1.0;
    public static double DISTANCE_TOLERANCE_CM = 2.0;
    public static double SPIN_TOLERANCE_DEGREES = 2.0;

    public static double ALIGNMENT_KP = 0.03;
    public static double APPROACH_KP = 0.02;
    public static double SPIN_KP = 0.4;

    // --- Poses para o Pedro Pathing (em CM) ---
    private final Pose START_POSE = new Pose(0, 0, Math.toRadians(0));
    private final Pose FORWARD_PATH_END_POSE = new Pose(200, 0, Math.toRadians(0)); // Move 2 metros para frente

    private PathChain pathToDriveForward;
    private Telemetry telemetryA;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();
        buildPaths();

        telemetryA.addData("Status", "Inicialização Completa. Pronto para iniciar!");
        telemetryA.update();

        waitForStart();

        if (opModeIsActive() &&!isStopRequested()) {

            // --- PASSO 1: NAVEGAR PARA FRENTE ATÉ VER UM ALVO ---
            telemetryA.addData("Passo", "1: Navegando para frente, procurando por alvos...");
            telemetryA.update();
            follower.followPath(pathToDriveForward, true);
            waitForPathToFinishOrTargetVisible();

            // Se o loop terminou porque um alvo foi visto, o follower foi interrompido.
            // Se terminou porque o caminho acabou, o robô para.

            // --- PASSO 2: ALINHAR COM O ALVO ENCONTRADO ---
            telemetryA.addData("Passo", "2: Alvo encontrado! Alinhando...");
            telemetryA.update();
            alignWithTarget();

            // --- PASSO 3: APROXIMAR DO ALVO ---
            telemetryA.addData("Passo", "3: Alinhado! Aproximando do Alvo (Alvo: " + TARGET_DISTANCE_CM + " cm)");
            telemetryA.update();
            approachTarget();

            // --- PASSO 4: EXECUTAR O GIRO ---
            telemetryA.addData("Passo", "4: Na distância correta! Executando Giro de 90 Graus");
            telemetryA.update();
            performSpin(90);

            // --- FIM (ou continue para o próximo passo) ---
            telemetryA.addData("Estado", "Primeiro ciclo concluído!");
            telemetryA.update();
            setDrivetrainPower(0, 0); // Garante que o robô pare
            sleep(2000); // Pausa para observação
        }
    }

    // --- MÉTODOS DE INICIALIZAÇÃO ---
    private void initializeHardware() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        // IMPORTANTE: Configure as direções dos motores para seu drivetrain
        // Ex: rightFront.setDirection(DcMotor.Direction.REVERSE);
        //     rightBack.setDirection(DcMotor.Direction.REVERSE);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(logoDirection, usbDirection)));
        imu.resetYaw();

        limelight.pipelineSwitch(0);
        limelight.start();

        follower.setStartingPose(START_POSE);
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    private void buildPaths() {
        pathToDriveForward = follower.pathBuilder()
                .addPath(new BezierLine(new Point(START_POSE), new Point(FORWARD_PATH_END_POSE)))
                .setLinearHeadingInterpolation(START_POSE.getHeading(), FORWARD_PATH_END_POSE.getHeading())
                .build();
    }

    // --- MÉTODOS DE CONTROLE E LÓGICA ---

    private void correctPoseWithVision() {
        limelight.updateRobotOrientation(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        LLResult result = limelight.getLatestResult();
        if (result == null ||!result.isValid()
                | result.getStaleness() > 200) return;

        Pose3D visionPose3D = result.getBotpose_MT2();
        if (visionPose3D!= null) {
            Pose visionPose = new Pose(
                    visionPose3D.getPosition().x * 100.0,
                    visionPose3D.getPosition().y * 100.0,
                    visionPose3D.getOrientation().getYaw(AngleUnit.RADIANS)
            );
            follower.setPose(visionPose);
        }
    }

    private boolean isTargetVisible() {
        LLResult result = limelight.getLatestResult();
        if (result == null ||!result.isValid()) return false;

        if (result.getFiducialResults()!= null &&!result.getFiducialResults().isEmpty()) {
            for (FiducialResult fiducial : result.getFiducialResults()) {
                if (TARGET_APRILTAG_IDS.contains(fiducial.getFiducialId())) {
                    return true; // Encontrou uma das tags da nossa lista!
                }
            }
        }
        return false;
    }

    private void alignWithTarget() {
        ElapsedTime timeout = new ElapsedTime();
        while(opModeIsActive() && timeout.seconds() < 3.0) {
            LLResult result = limelight.getLatestResult();
            if (!isTargetVisible()) {
                setDrivetrainPower(0, 0);
                continue;
            }

            double tx = result.getTx();
            if (Math.abs(tx) < ALIGNMENT_TOLERANCE_DEGREES) {
                setDrivetrainPower(0, 0);
                return; // Alinhado!
            }

            double turnPower = -tx * ALIGNMENT_KP;
            setDrivetrainPower(0, turnPower);
            updateTelemetry();
        }
        setDrivetrainPower(0, 0);
    }

    private void approachTarget() {
        ElapsedTime timeout = new ElapsedTime();
        while(opModeIsActive() && timeout.seconds() < 4.0) {
            LLResult result = limelight.getLatestResult();
            if (!isTargetVisible()) {
                setDrivetrainPower(0, 0);
                continue;
            }

            double currentDistance = calculateDistanceToTarget(result.getTy());
            if (currentDistance < 0) {
                setDrivetrainPower(0, 0);
                continue;
            }

            double error = currentDistance - TARGET_DISTANCE_CM;
            if (Math.abs(error) < DISTANCE_TOLERANCE_CM) {
                setDrivetrainPower(0, 0);
                return; // Na distância correta!
            }

            double forwardPower = error * APPROACH_KP;
            setDrivetrainPower(forwardPower, 0);
            updateTelemetry();
        }
        setDrivetrainPower(0, 0);
    }

    private void performSpin(double degrees) {
        double startAngleRad = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double targetAngleRad = AngleUnit.normalizeRadians(startAngleRad + Math.toRadians(degrees));
        ElapsedTime spinTimer = new ElapsedTime();

        while (opModeIsActive() && spinTimer.seconds() < 4.0) {
            double currentAngleRad = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double errorRad = AngleUnit.normalizeRadians(targetAngleRad - currentAngleRad);

            if (Math.abs(errorRad) < Math.toRadians(SPIN_TOLERANCE_DEGREES)) {
                break;
            }

            double turnPower = errorRad * SPIN_KP;
            setDrivetrainPower(0, turnPower);
            updateTelemetry();
        }
        setDrivetrainPower(0, 0);
    }

    // --- MÉTODOS UTILITÁRIOS ---

    private double calculateDistanceToTarget(double ty) {
        // Valores que você forneceu e ajustou
        double cameraHeightCM = 14.0;
        double targetHeightCM = 10.16; // 101.6mm
        double cameraPitchRadians = Math.toRadians(0.0); // Câmera paralela ao chão

        double angleToTargetRadians = Math.toRadians(ty) + cameraPitchRadians;

        // Se o alvo está abaixo da horizontal da câmera, ty é negativo, e o ângulo total também.
        // tan(ângulo negativo) é negativo. A diferença de altura (10.16 - 14.0) também é negativa.
        // Negativo / Negativo = Positivo, então a distância é correta.
        if (angleToTargetRadians < 0) {
            return (targetHeightCM - cameraHeightCM) / Math.tan(angleToTargetRadians);
        }
        return -1; // Retorna distância inválida se o alvo estiver acima da horizontal da câmera
    }

    // Método para controlar um drivetrain mecanum
    private void setDrivetrainPower(double forward, double turn) {
        double leftFrontPower  = forward - turn;
        double leftBackPower   = forward - turn;
        double rightFrontPower = forward + turn;
        double rightBackPower  = forward + turn;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));
        if (max > 1.0) {
            leftFrontPower  /= max;
            leftBackPower   /= max;
            rightFrontPower /= max;
            rightBackPower  /= max;
        }

        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);
    }

    private void waitForPathToFinish() {
        while (opModeIsActive() &&!isStopRequested() && follower.isBusy()) {
            correctPoseWithVision();
            follower.update();
            updateTelemetry();
            idle();
        }
    }

    private void waitForPathToFinishOrTargetVisible() {
        ElapsedTime timeout = new ElapsedTime();
        while (opModeIsActive() &&!isStopRequested() && follower.isBusy() && timeout.seconds() < 7.0) {
            correctPoseWithVision();
            follower.update();
            if (isTargetVisible()) {
                follower.breakFollowing();
                break;
            }
            updateTelemetry();
            idle();
        }
    }

    private void updateTelemetry() {
        telemetryA.addData("Pose do Robô (cm)", follower.getPose().toString());

        LLResult result = limelight.getLatestResult();
        if (result!= null && result.isValid()) {
            telemetryA.addData("Limelight Alvo Visível", isTargetVisible()? "Sim (ID Correto)" : "Sim (Outro ID)");
            telemetryA.addData("Limelight tx", "%.2f", result.getTx());
            telemetryA.addData("Distância Calculada (cm)", "%.2f", calculateDistanceToTarget(result.getTy()));
        } else {
            telemetryA.addData("Limelight Alvo Visível", "Não");
        }
        telemetryA.update();
    }
}