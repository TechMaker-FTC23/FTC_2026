// Nome do arquivo: PoseFusionTest_Improved.java
package pedroPathing.examples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import pedroPathing.util.KalmanFilter2D;

@Config
@TeleOp
public class PoseFusionTest extends LinearOpMode {

    private Telemetry telemetryA;
    private Limelight3A limelight;
    private IMU imu;
    private Follower follower;
    private FtcDashboard dashboard;

    // Nosso filtro de fusão
    private KalmanFilter2D poseFilter;

    public final Pose START_POSE = new Pose(0, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        // --- INICIALIZAÇÃO ---
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        dashboard = FtcDashboard.getInstance();
        imu = hardwareMap.get(IMU.class, "imu");
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Configuração da IMU
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(logoDirection, usbDirection)));
        imu.resetYaw();

        // Configuração da Limelight
        limelight.pipelineSwitch(0);
        limelight.start();

        // Inicializa o follower e o nosso filtro com a mesma pose inicial
        follower.setStartingPose(START_POSE);
        poseFilter = new KalmanFilter2D(START_POSE);

        telemetryA.addData("Status", "Inicialização Completa. Pronto para dirigir!");
        telemetryA.update();

        waitForStart();
        follower.startTeleopDrive();
        // --- LOOP DE TELEOP ---
        Pose fusedPose = new Pose();
        while (opModeIsActive()) {
            // --- ATUALIZAÇÕES DE SENSORES E FILTRO ---
            follower.update();

           // 1. Atualiza o follower com os dados do OTOS/IMU. Isso nos dá a pose da odometria.
            Pose odometryPose = follower.getPose();
            // 3. Obtém a pose da visão da Limelight (se disponível)
            Pose visionPose = getVisionPose();
            // 2. Chama a etapa de PREDIÇÃO do filtro com a nova pose da odometria.
            if(!(odometryPose.getX()==0 && odometryPose.getY()==0)) {
                odometryPose = fusedPose;
            }
            poseFilter.predict(odometryPose);


            if (visionPose != null) {


                poseFilter.update(visionPose);
                // 5. Pega a pose final fundida do filtro. Esta é a nossa "verdade absoluta".


            }
            fusedPose = poseFilter.getEstimate();
            follower.setPose(fusedPose);



            // --- CONTROLE DO DRIVETRAIN ---
            // Em vez de resetar a pose do follower, nós implementamos o field-centric manualmente
            // usando a orientação (heading) da nossa pose fundida, que é mais precisa.

            // Pega os comandos do joystick
            double yInput = -gamepad1.left_stick_y; // Frente/Trás
            double xInput = -gamepad1.left_stick_x; // Esquerda/Direita (Strafe)
            double turnInput = -gamepad1.right_stick_x; // Rotação

            // Envia os vetores rotacionados para o follower em modo ROBOT-CENTRIC.
            follower.setTeleOpMovementVectors(yInput, xInput, turnInput, false);

           /* if(gamepad1.start){
                imu.resetYaw();
                follower.setPose(START_POSE);
                odometryPose = START_POSE;
                fusedPose = START_POSE;
            }*/
            // --- PLOTAGEM E TELEMETRIA ---
            // Agora, a plotagem mostrará claramente o drift da odometria (vermelho)
            // e como a pose fundida (azul) se mantém correta.
            plotPosesToDashboard(odometryPose, visionPose, fusedPose);
        }

        limelight.stop();
    }

    /**
     * Obtém a pose da Limelight e a converte para o formato do Pedro Pathing.
     * Retorna null se nenhuma pose confiável for encontrada.
     */
    private Pose getVisionPose() {
        limelight.updateRobotOrientation(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        //limelight.updateRobotOrientation(follower.getPose().getHeading());
        LLResult result = limelight.getLatestResult();

        if (result == null ||!result.isValid()
                | result.getStaleness() > 200) {
            return null;
        }

        Pose3D visionPose3D = result.getBotpose_MT2();
        if (visionPose3D!= null) {
            return new Pose(
                    visionPose3D.getPosition().x * 100.0, // Converte metros para CM
                    visionPose3D.getPosition().y * 100.0,
                    visionPose3D.getOrientation().getYaw(AngleUnit.DEGREES)
            );
        }
        return null;
    }

    /**
     * Desenha as três poses (Odometria, Visão, Fundida) no FTC Dashboard para visualização.
     */
    private void plotPosesToDashboard(Pose odoPose, Pose visionPose, Pose fusedPose) {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        packet.put("Odo X (cm)", odoPose.getX());
        packet.put("Odo Y (cm)", odoPose.getY());
        packet.put("Odo H (deg)", Math.toDegrees(odoPose.getHeading()));

        // Desenha a pose da odometria (OTOS) em vermelho
        drawRobotOnCanvas(fieldOverlay, odoPose, "red");

        if (visionPose!= null) {
            packet.put("Vision X (cm)", visionPose.getX());
            packet.put("Vision Y (cm)", visionPose.getY());
            packet.put("Vision H (deg)", Math.toDegrees(visionPose.getHeading()));
            // Desenha a pose da visão (Limelight) em verde
            drawRobotOnCanvas(fieldOverlay, visionPose, "lime");
        }

        packet.put("Fused X (cm)", fusedPose.getX());
        packet.put("Fused Y (cm)", fusedPose.getY());
        packet.put("Fused H (deg)", Math.toDegrees(fusedPose.getHeading()));
        // Desenha a pose final fundida em azul
        drawRobotOnCanvas(fieldOverlay, fusedPose, "blue");

        dashboard.sendTelemetryPacket(packet);
    }

    /**
     * Função auxiliar para desenhar um robô no canvas do dashboard.
     */
    private void drawRobotOnCanvas(Canvas canvas, Pose robotPose, String color) {
        if (robotPose == null) return;
        double xCm = robotPose.getX();
        double yCm = robotPose.getY();
        double headingRad = robotPose.getHeading();

        // Converte CM para Polegadas para plotagem no dashboard
        double xInches = xCm / 2.54;
        double yInches = yCm / 2.54;

        canvas.setStroke(color);
        canvas.strokeCircle(xInches, yInches, 8); // Raio de 8 polegadas (aprox. 18x18)
        double arrowX = Math.cos(headingRad) * 10; // Comprimento da seta em polegadas
        double arrowY = Math.sin(headingRad) * 10;
        double x2 = xInches + arrowX;
        double y2 = yInches + arrowY;
        canvas.strokeLine(xInches, yInches, x2, y2);
    }
}