package techmaker.examples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import techmaker.constants.FConstants;
import techmaker.constants.LConstants;

@TeleOp(name = "PathChainTest (Com Rotinas)")
public class PathChainTest extends OpMode {
    private Telemetry telemetryA;
    private Follower follower;

    // Posições de jogo para as rotinas de autônomo.
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose Coletacima = new Pose(95, -80, Math.toRadians(0));
    private final Pose voltaEntrga = new Pose(40, -20, Math.toRadians(135));
    private final Pose ColetaMeio = new Pose(100, -30, Math.toRadians(-90));
    private final Pose entrega = new Pose(-20, -20, Math.toRadians(135));

    // Variáveis de controlo para detetar uma única pressão de botão
    private boolean dpadRightPressed = false;
    private boolean dpadLeftPressed = false;
    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;

    @Override
    public void init() {
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        telemetryA.addLine("Robô inicializado. Pressione D-Pad para testar as rotinas.");
        telemetryA.update();
    }

    @Override
    public void start() {
        // Nenhum comando de movimento no start.
    }

    @Override
    public void loop() {
        follower.update();

        // Só permite iniciar uma nova rotina se o robô não estiver ocupado.
        if (!follower.isBusy()) {

            // --- ROTINA 1: Ciclo de Coleta de Cima (D-Pad Direita) ---
            // Vai para a coleta de cima e depois retorna para a área de entrega.
            if (gamepad1.dpad_right && !dpadRightPressed) {
                PathChain cicloColetaCima = follower.pathBuilder()
                        .addPath(new BezierLine(new Point(follower.getPose()), new Point(Coletacima)))
                        .addPath(new BezierLine(new Point(Coletacima), new Point(entrega)))
                        .build();
                follower.followPath(cicloColetaCima, true);
            }

            // --- ROTINA 2: Ciclo de Coleta do Meio (D-Pad Esquerda) ---
            // Vai para a coleta do meio e depois retorna para a área de entrega.
            if (gamepad1.dpad_left && !dpadLeftPressed) {
                PathChain cicloColetaMeio = follower.pathBuilder()
                        .addPath(new BezierLine(new Point(follower.getPose()), new Point(ColetaMeio)))
                        .addPath(new BezierLine(new Point(ColetaMeio), new Point(entrega)))
                        .build();
                follower.followPath(cicloColetaMeio, true);
            }

            // --- ROTINA 3: Caminho Complexo (D-Pad Cima) ---
            // Demonstra um caminho com múltiplos pontos intermédios.
            if (gamepad1.dpad_up && !dpadUpPressed) {
                PathChain cicloComplexo = follower.pathBuilder()
                        .addPath(new BezierLine(new Point(follower.getPose()), new Point(voltaEntrga)))
                        .addPath(new BezierLine(new Point(voltaEntrga), new Point(Coletacima)))
                        .addPath(new BezierLine(new Point(Coletacima), new Point(entrega)))
                        .build();
                follower.followPath(cicloComplexo, true);
            }

            // --- ROTINA 4: Retornar ao Início (D-Pad Baixo) ---
            // Um comando útil para resetar a posição do robô durante os testes.
            if (gamepad1.dpad_down && !dpadDownPressed) {
                PathChain pathToStart = follower.pathBuilder()
                        .addPath(new BezierLine(new Point(follower.getPose()), new Point(startPose)))
                        .build();
                follower.followPath(pathToStart, true);
            }
        }

        // Atualiza o estado dos botões no final do loop.
        dpadRightPressed = gamepad1.dpad_right;
        dpadLeftPressed = gamepad1.dpad_left;
        dpadUpPressed = gamepad1.dpad_up;
        dpadDownPressed = gamepad1.dpad_down;

        // Envia a telemetria para o Dashboard.
        follower.telemetryDebug(telemetryA);
        telemetryA.addData("Está Ocupado?", follower.isBusy());
        telemetryA.update();
    }
}
