package techmaker.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import techmaker.constants.FConstants;
import techmaker.constants.LConstants;
import techmaker.subsystems.ClawSubsystem;
import techmaker.subsystems.ElevatorSubsystem;
import techmaker.subsystems.IntakeSubsystem;
import techmaker.util.StateMachine;

@Autonomous
public class autonomoteste extends LinearOpMode {

    private Telemetry telemetryA;
    private Follower follower;
    private Limelight3A limelight3A;
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose BargeUp = new Pose(95/2.54, 80/2.54, Math.toRadians(0));
    private final Pose BargeMiddle = new Pose(100/2.54, 30/2.54, Math.toRadians(-90));
    private final Pose SpikeMarkC = new Pose(-48.4/2.54,63/2.54, Math.toRadians(0));
    private final Pose SpikeMarkD = new Pose(-23/2.54,63/2.54, Math.toRadians(0));
    private final Pose SpikeMarkE = new Pose(-48.4/2.54,63/2.54, Math.toRadians(30));
    private final Pose MidBasket = new Pose(-10/2.54,20/2.54, Math.toRadians(135));
    private final Pose Basket = new Pose(-20/2.54, 5/2.54, Math.toRadians(135));

    @Override
    public void runOpMode() throws InterruptedException {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(0);
        limelight3A.start();
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addData("Status", "Autônomo Inicializado.");
        telemetryA.addData("Sequência", "START -> MEIO -> ENTREGA -> MEIO -> ENTREGA -> CIMA -> ENTREGA");
        telemetryA.update();

        waitForStart();

        // --- EXECUÇÃO DO AUTÔNOMO ---
        if (opModeIsActive() && !isStopRequested()) {


            // --- CICLO 1: START -> MID POINT -> ENTREGA ---
            telemetryA.addData("CICLO 1", "Entregando o pre colocado");
            telemetryA.update();
            executePathToPose(MidBasket, true, null); // Caminho reto

            // AQUI: Preciso Adicionar o código para os mecanismos COLETAREM


            telemetryA.addData("CICLO 1", "Retornando para Entrega");
            telemetryA.update();
            executePathToPose(Basket, true, null); // Caminho reto

            // AQUI: Preciso Adicionar o código para os mecanismos PONTUAREM

            sleep(500);

            // --- CICLO 2: ENTREGA -> COLETA MEIO -> ENTREGA ---
            telemetryA.addData("CICLO 2", "Indo para o SpikeMark C");
            telemetryA.update();
            executePathToPose(SpikeMarkC, false, null);

            // AQUI: Preciso Adicionar o código para os mecanismos COLETAREM

            sleep(500);

            telemetryA.addData("CICLO 2", "Retornando para Entrega");
            telemetryA.update();
            executePathToPose(Basket, false, null); // Caminho reto

            // AQUI: Preciso Adicionar o código para os mecanismos PONTUAREM
            sleep(500);

            // --- CICLO 3: ENTREGA -> COLETA CIMA -> ENTREGA ---
            telemetryA.addData("CICLO 3", "Indo para o SpikeMark D");
            telemetryA.update();
            executePathToPose(SpikeMarkD, false, null); // Caminho reto

            // AQUI: Preciso Adicionar o código para os mecanismos COLETAREM
            sleep(500);

            telemetryA.addData("CICLO 3", "Retornando para Entrega");
            telemetryA.update();
            executePathToPose(Basket, false, null); // Caminho reto

            // AQUI: Preciso Adicionar o código para os mecanismos PONTUAREM
            sleep(500);

            telemetryA.addData("CICLO 4", "Retornando para Spike Mark E");
            telemetryA.update();
            executePathToPose(SpikeMarkE, false, null); // Caminho reto

            // AQUI: Preciso Adicionar o código para os mecanismos PONTUAREM
            sleep(500);

            telemetryA.addData("CICLO 4", "Retornando para Entrega Final");
            telemetryA.update();
            executePathToPose(Basket, false, null); // Caminho reto

            // AQUI: Preciso Adicionar o código para os mecanismos PONTUAREM
            sleep(500);

            telemetryA.addData("Status", "Autônomo Concluído!");
            telemetryA.update();
        }
    }

    /**
     * MÉTODO AUXILIAR 1: Constrói e executa um caminho.
     * Esta função pega um destino e cria um caminho da posição ATUAL do robô até lá.
     *
     * @param targetPose   A pose final desejada.
     * @param isCurve      Se o caminho deve ser uma curva (requer um controlPoint).
     * @param controlPoint O ponto de controle para a BezierCurve (ignorado se isCurve for false).
     */
    private void executePathToPose(Pose targetPose, boolean isCurve, Pose controlPoint) {
        if (!opModeIsActive()) return;

        Pose startOfPathPose = follower.getPose(); // Pega a pose atual para o início do caminho
        PathChain path;

        if (isCurve && controlPoint != null) {
            // Constrói um caminho em curva
            path = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Point(startOfPathPose),
                            new Point(controlPoint),
                            new Point(targetPose)))
                    .setLinearHeadingInterpolation(startOfPathPose.getHeading(), targetPose.getHeading())
                    .build();
        } else {
            // Constrói um caminho em linha reta
            path = follower.pathBuilder()
                    .addPath(new BezierLine(new Point(startOfPathPose), new Point(targetPose)))
                    .setLinearHeadingInterpolation(startOfPathPose.getHeading(), targetPose.getHeading())
                    .build();
        }

        follower.followPath(path, true); // holdEnd = true é bom para manter a posição
        waitForPathToFinish(); // Chama o nosso segundo método auxiliar para esperar
    }

    private void waitForPathToFinish() {
        while (opModeIsActive() && !isStopRequested() && follower.isBusy()) {
            follower.update(); // ESSENCIAL: Atualiza a lógica do seguidor de caminho
            follower.telemetryDebug(telemetryA); // Envia telemetria para o Dashboard
            telemetryA.update(); // Atualiza a telemetria na Driver Station
            idle(); // Cede tempo de CPU para outros processos do robô

            LLResult result = limelight3A.getLatestResult();

            Pose3D botpose = null;
            if (result != null && result.isValid()) {
                botpose = result.getBotpose_MT2();
            }
            if (botpose != null) {
                // O botpose retorna as coordenadas em metros.
                telemetryA.addData("Botpose X (metros)", "%.2f", botpose.getPosition().x);
                telemetryA.addData("Botpose Y (metros)", "%.2f", botpose.getPosition().y);
                telemetryA.addData("Botpose Z (metros)", "%.2f", botpose.getPosition().z);

                telemetryA.addData("Botpose Yaw (graus)", "%.2f", botpose.getOrientation().getYaw(AngleUnit.RADIANS));

                // Converte a Pose3D da Limelight (metros) para a Pose 2D do Pedro Pathing (CM)
                Pose visionPose = new Pose(
                        botpose.getPosition().x /0.0254,
                        botpose.getPosition().y /0.0254,
                        botpose.getOrientation().getYaw(AngleUnit.RADIANS)

                );

                follower.setPose(visionPose);
            }
        }
    }
}