package pedroPathing.examples; // Ou seu pacote desejado

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

// Supondo que suas classes de constantes FConstants e LConstants estão acessíveis
// e LConstants configura Pedro Pathing para usar DistanceUnit.CM
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "Pedro Auto CM Poses Customizadas Corrigido v2", group = "Pedro")
public class PedroAutonomousExample extends LinearOpMode {

    private Follower follower;
    private Limelight3A limelight;
    private FtcDashboard dashboard;
    private Telemetry telemetryA;

    private ElapsedTime limelightUpdateTimer = new ElapsedTime();
    private static final long LIMELIGHT_UPDATE_INTERVAL_MS = 250;
    private Pose lastKnownLimelightPose = null; // Para plotagem

    private final Pose START_POSE = new Pose(0, 0, Math.toRadians(0));
    private final Pose STAR_POSE = new Pose(1, 0, Math.toRadians(0));
    private final Pose COMECO_POSE = new Pose(2, 0, Math.toRadians(0));
    private final Pose COLETACIMA_POSE = new Pose(160, -5, Math.toRadians(270));
    private final Pose VOLTAENTREGA_CONTROL_POSE = new Pose(40, 20, Math.toRadians(135));
    private final Pose COLETAMEIO_POSE = new Pose(80, -75, Math.toRadians(180));
    private final Pose ENTREGA_POSE = new Pose(20, 30, Math.toRadians(135));

    @Override
    public void runOpMode() throws InterruptedException {
        // Inicialização
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class); // Assumes LConstants sets units to CM
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // Use o pipeline configurado para AprilTags [1]
        limelight.setPollRateHz(30); // Ajuste conforme necessário [1]
        limelight.start(); // [1]

        dashboard = FtcDashboard.getInstance();
        telemetryA = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());

        telemetryA.addData("Status", "Inicializando (Unidades: CM)...");
        telemetryA.update();

        Pose initialSystemPose = START_POSE;
        ElapsedTime initTimer = new ElapsedTime();
        while (initTimer.seconds() < 3.0 && opModeInInit()) { // Tenta por 3 segundos durante a inicialização
            Pose llPose = getLimelightRobotPoseCm();
            if (llPose!= null) {
                initialSystemPose = llPose;
                lastKnownLimelightPose = llPose;
                telemetryA.addData("Status", "Pose inicial definida pela Limelight!");
                break;
            }
            sleep(50);
        }
        follower.setStartingPose(initialSystemPose);
        if (lastKnownLimelightPose == null) {
            telemetryA.addData("Status", "Limelight não encontrou pose inicial, usando padrão.");
        }
        telemetryA.update();

        telemetryA.addData("Status", "Pronto para iniciar!");
        telemetryA.addData("Pose Inicial do Follower (cm)", follower.getPose().toString());
        telemetryA.update();

        waitForStart();

        if (opModeIsActive() &&!isStopRequested()) {
            limelightUpdateTimer.reset();

            // Sequência Autônoma usando suas poses:
            executePath("Caminho para StarPose", STAR_POSE, true, false, null);
            executePath("Caminho para ComecoPose", COMECO_POSE, true, false, null);
            executePath("Caminho para EntregaPose", ENTREGA_POSE, true, false, null);

            executePath("Caminho para ColetaMeioPose", COLETAMEIO_POSE, true, false, null);
            executePath("Caminho para ColetaCimaPose", COLETACIMA_POSE, true, false, null);

            // telemetryA.addData("Ação", "Coletando...");
            // telemetryA.update();
            // sleep(1000); // Simula ação de coleta

            // Para o caminho de curva, precisamos do ponto de controle
            executePath("Retornando ao Início (Curva)", START_POSE, false, true, VOLTAENTREGA_CONTROL_POSE);


            telemetryA.addData("Status", "Autônomo Concluído!");
            telemetryA.update();
        }
    }

    // Método auxiliar para executar um caminho
    private void executePath(String pathName, Pose targetEndPose, boolean holdEnd, boolean isCurve, Pose controlPoint) {
        if (!opModeIsActive()
                | isStopRequested()) return;

        Pose currentActualPose = follower.getPose();

        telemetryA.addData("Executando", pathName);
        telemetryA.addData("De (cm)", currentActualPose.toString());
        telemetryA.addData("Para (cm)", targetEndPose.toString());
        telemetryA.update();

        PathChain currentPathSegment;
        if (isCurve && controlPoint!= null) {
            currentPathSegment = follower.pathBuilder()
                    .addPath(new BezierCurve(new Point(currentActualPose), new Point(controlPoint), new Point(targetEndPose)))
                    .setLinearHeadingInterpolation(currentActualPose.getHeading(), targetEndPose.getHeading())
                    .build();
        } else {
            currentPathSegment = follower.pathBuilder()
                    .addPath(new BezierLine(new Point(currentActualPose), new Point(targetEndPose)))
                    .setLinearHeadingInterpolation(currentActualPose.getHeading(), targetEndPose.getHeading())
                    .build();
        }

        follower.followPath(currentPathSegment, holdEnd); // [2]
        waitForPathToCompleteWithLimelightUpdates();
    }

    // Método para obter a pose da Limelight e converter para Pose do Pedro Pathing (CM, radianos)
    private Pose getLimelightRobotPoseCm() {
        LLResult result = limelight.getLatestResult();
        if (result!= null && result.isValid() && result.getStaleness() < 200) {
            Pose3D botPose3D = result.getBotpose_MT2();
            if (botPose3D!= null) {
                Position pos = botPose3D.getPosition();
                YawPitchRollAngles ori = botPose3D.getOrientation();

                // Converte metros para CM
                double xCm = pos.x * 100.0;
                double yCm = pos.y * 100.0;

                // Obtém yaw em radianos
                double headingRadians = ori.getYaw(AngleUnit.RADIANS); // [3]

                lastKnownLimelightPose = new Pose(xCm, yCm, headingRadians);
                return lastKnownLimelightPose;
            }
        }
        lastKnownLimelightPose = null;
        return null;
    }

    private void waitForPathToCompleteWithLimelightUpdates() {
        while (opModeIsActive() &&!isStopRequested() && follower.isBusy()) {
            if (limelightUpdateTimer.milliseconds() >= LIMELIGHT_UPDATE_INTERVAL_MS) {
                Pose visionPoseCm = getLimelightRobotPoseCm();
                if (visionPoseCm!= null) {
                    follower.setPose(visionPoseCm);
                    // follower.updatePose(); // Verifique se é necessário para sua versão/configuração do Pedro Pathing
                    telemetryA.addData("Correção Limelight (cm)", visionPoseCm.toString());
                }
                limelightUpdateTimer.reset();
            }

            follower.update();

            plotCurrentStateToDashboard();
            telemetryA.update();
            idle();
        }
        follower.update();
        plotCurrentStateToDashboard();
        telemetryA.update();
    }

    private void plotCurrentStateToDashboard() {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay(); // [5]
        Pose currentFollowerPoseCm = follower.getPose();

        drawRobotOnCanvas(fieldOverlay, currentFollowerPoseCm, "blue", 7.0); // Raio em polegadas para o dashboard

        if (lastKnownLimelightPose!= null) {
            drawRobotOnCanvas(fieldOverlay, lastKnownLimelightPose, "green", 5.0); // Raio em polegadas
        }

        packet.put("Follower X (cm)", String.format("%.2f", currentFollowerPoseCm.getX()));
        packet.put("Follower Y (cm)", String.format("%.2f", currentFollowerPoseCm.getY()));
        packet.put("Follower H (rad)", String.format("%.2f", currentFollowerPoseCm.getHeading()));
        if (lastKnownLimelightPose!= null) {
            packet.put("Limelight X (cm)", String.format("%.2f", lastKnownLimelightPose.getX()));
            packet.put("Limelight Y (cm)", String.format("%.2f", lastKnownLimelightPose.getY()));
            packet.put("Limelight H (rad)", String.format("%.2f", lastKnownLimelightPose.getHeading()));
        }
        dashboard.sendTelemetryPacket(packet);
    }

    private void drawRobotOnCanvas(Canvas canvas, Pose robotPoseCm, String color, double radiusInches) {
        if (robotPoseCm == null) return;

        double xInches = robotPoseCm.getX() / 2.54;
        double yInches = robotPoseCm.getY() / 2.54;
        double headingRadians = robotPoseCm.getHeading();

        canvas.setFill(color);
        canvas.fillCircle(xInches, yInches, radiusInches);

        double lineLengthInches = radiusInches * 1.5;
        double x2Inches = xInches + Math.cos(headingRadians) * lineLengthInches;
        double y2Inches = yInches + Math.sin(headingRadians) * lineLengthInches;
        canvas.setStroke("black");
        canvas.setStrokeWidth(1);
        canvas.strokeLine(xInches, yInches, x2Inches, y2Inches);
    }
    
    public static Pose weightedAveragePose(Pose odoPose, Pose visionPose, double odoWeight) {
        double visionWeight = 1.0 - odoWeight;
        double x = odoPose.getX() * odoWeight + visionPose.getX() * visionWeight;
        double y = odoPose.getY() * odoWeight + visionPose.getY() * visionWeight;
        double odoHeading = odoPose.getHeading();
        double visionHeading = visionPose.getHeading();
        double delta = angleWrap(visionHeading - odoHeading);
        double heading = odoHeading + delta * visionWeight;
        return new Pose(x, y, heading);
    }

    public static double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}