package pedroPathing.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous
public class autonomoteste extends LinearOpMode {

    private Telemetry telemetryA;
    private Follower follower;
    
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose StarPose = new Pose(1, 0, Math.toRadians(0)); // Não usado na sequência, mas mantido
    private final Pose Começo = new Pose(2, 0, Math.toRadians(0));     // Não usado na sequência, mas mantido
    private final Pose Coletacima = new Pose(160, -5, Math.toRadians(270));
    private final Pose voltaEntrga = new Pose(40, 20, Math.toRadians(135)); // Usado como ponto de controle
    private final Pose ColetaMeio = new Pose(80, -75, Math.toRadians(180));
    private final Pose entrega = new Pose(20, 30, Math.toRadians(135));


    @Override
    public void runOpMode() throws InterruptedException {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addData("Status", "Autônomo Inicializado.");
        telemetryA.addData("Sequência", "START -> MEIO -> ENTREGA -> MEIO -> ENTREGA -> CIMA -> ENTREGA");
        telemetryA.update();

        waitForStart();

        // --- EXECUÇÃO DO AUTÔNOMO ---
        if (opModeIsActive() &&!isStopRequested()) {

            // --- CICLO 1: START -> COLETA MEIO -> ENTREGA ---
            telemetryA.addData("CICLO 1", "Indo para Coleta Meio");
            telemetryA.update();
            executePathToPose(ColetaMeio, false, null); // Caminho reto

            // AQUI: Preciso Adicionar o código para os mecanismos COLETAREM


            sleep(500); // Pausa para a ação do mecanismo

            telemetryA.addData("CICLO 1", "Retornando para Entrega");
            telemetryA.update();
            executePathToPose(entrega, false, null); // Caminho reto

            // AQUI: Preciso Adicionar o código para os mecanismos PONTUAREM

            sleep(500);

            // --- CICLO 2: ENTREGA -> COLETA MEIO -> ENTREGA ---
            telemetryA.addData("CICLO 2", "Indo para Coleta Meio");
            telemetryA.update();
            executePathToPose(ColetaMeio, false, null);

            // AQUI: Preciso Adicionar o código para os mecanismos COLETAREM

            sleep(500);

            telemetryA.addData("CICLO 2", "Retornando para Entrega");
            telemetryA.update();
            executePathToPose(entrega, false, null); // Caminho reto

            // AQUI: Preciso Adicionar o código para os mecanismos PONTUAREM
            sleep(500);

            // --- CICLO 3: ENTREGA -> COLETA CIMA -> ENTREGA ---
            telemetryA.addData("CICLO 3", "Indo para Coleta Cima");
            telemetryA.update();
            executePathToPose(Coletacima, false, null); // Caminho reto

            // AQUI: Preciso Adicionar o código para os mecanismos COLETAREM
            sleep(500);

            telemetryA.addData("CICLO 3", "Retornando para Entrega Final");
            telemetryA.update();
            executePathToPose(entrega, false, null); // Caminho reto

            // AQUI: Preciso Adicionar o código para os mecanismos PONTUAREM
            sleep(500);

            telemetryA.addData("Status", "Autônomo Concluído!");
            telemetryA.update();
        }
    }

    /**
     * MÉTODO AUXILIAR 1: Constrói e executa um caminho.
     * Esta função pega um destino e cria um caminho da posição ATUAL do robô até lá.
     * @param targetPose A pose final desejada.
     * @param isCurve Se o caminho deve ser uma curva (requer um controlPoint).
     * @param controlPoint O ponto de controle para a BezierCurve (ignorado se isCurve for false).
     */
    private void executePathToPose(Pose targetPose, boolean isCurve, Pose controlPoint) {
        if (!opModeIsActive()) return;

        Pose startOfPathPose = follower.getPose(); // Pega a pose atual para o início do caminho
        PathChain path;

        if (isCurve && controlPoint!= null) {
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
        while (opModeIsActive() &&!isStopRequested() && follower.isBusy()) {
            follower.update(); // ESSENCIAL: Atualiza a lógica do seguidor de caminho
            follower.telemetryDebug(telemetryA); // Envia telemetria para o Dashboard
            telemetryA.update(); // Atualiza a telemetria na Driver Station
            idle(); // Cede tempo de CPU para outros processos do robô
        }
    }
}