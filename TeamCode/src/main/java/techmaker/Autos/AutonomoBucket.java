package techmaker.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import techmaker.constants.FConstants;
import techmaker.constants.LConstants;
@Disabled

@Autonomous
public class AutonomoBucket extends LinearOpMode {

    private Telemetry telemetryA;
    private Follower follower;

    // --- Suas Poses Originais ---
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose StarPose = new Pose(1, 0, Math.toRadians(0));
    private final Pose Começo = new Pose(2, 0, Math.toRadians(0));
    private final Pose Coletacima = new Pose(160, -5, Math.toRadians(270));
    private final Pose voltaEntrga = new Pose(40, 20, Math.toRadians(135));
    private final Pose ColetaMeio = new Pose(80, -75, Math.toRadians(180));
    private final Pose entrega = new Pose(20, 30, Math.toRadians(135));

    private PathChain pathToColetaMeio_fromStart;
    private PathChain pathToColetaMeio_fromEntrega;
    private PathChain pathToEntrega_fromColetaMeio;
    private PathChain pathToColetaCima_fromEntrega;
    private PathChain pathToEntrega_fromColetaCima;
    private PathChain pathToComeco_fromEntrega;

    @Override
    public void runOpMode() throws InterruptedException {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        // Constrói todos os caminhos que vamos usar uma única vez
        buildAllPaths();

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addData("Status", "Autônomo Seguro Inicializado.");
        telemetryA.addData("Info", "Todos os caminhos foram pré-construídos.");
        telemetryA.update();

        waitForStart();

        if (opModeIsActive() &&!isStopRequested()) {

            telemetryA.addData("Passo 1", "Indo para 'ColetaMeio'");
            telemetryA.update();
            follower.followPath(pathToColetaMeio_fromStart, true);
            waitForPathToFinish();

            sleep(500);

            telemetryA.addData("Passo 2", "Indo para 'Entrega'");
            telemetryA.update();
            follower.followPath(pathToEntrega_fromColetaCima, true);
            waitForPathToFinish();

            sleep(500);

            telemetryA.addData("Passo 3", "Retornando para 'Coleta'");
            telemetryA.update();
            follower.followPath(pathToColetaCima_fromEntrega, true);
            waitForPathToFinish();

            sleep(500);

            telemetryA.addData("Passo 4", "Indo para 'Entrga'");
            telemetryA.update();
            follower.followPath(pathToEntrega_fromColetaMeio, true);
            waitForPathToFinish();

            sleep(500);

            telemetryA.addData("Passo 5", "Indo para 'ColetaCima'");
            telemetryA.update();
            follower.followPath(pathToColetaCima_fromEntrega, true);
            waitForPathToFinish();

            sleep(500);

            telemetryA.addData("Passo 6", "Estacionando no 'Começo' (Curva)");
            telemetryA.update();
            follower.followPath(pathToEntrega_fromColetaCima, false);
            waitForPathToFinish();

            telemetryA.addData("Status", "Autônomo Concluído!");
            telemetryA.update();
        }
    }

    private void buildAllPaths() {

        pathToColetaMeio_fromStart = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(ColetaMeio)))
                .setLinearHeadingInterpolation(startPose.getHeading(), ColetaMeio.getHeading())
                .build();

        pathToColetaMeio_fromEntrega = follower.pathBuilder()
                .addPath(new BezierLine(new Point(entrega), new Point(ColetaMeio)))
                .setLinearHeadingInterpolation(entrega.getHeading(), ColetaMeio.getHeading())
                .build();

        pathToEntrega_fromColetaMeio = follower.pathBuilder()
                .addPath(new BezierLine(new Point(ColetaMeio), new Point(entrega)))
                .setLinearHeadingInterpolation(ColetaMeio.getHeading(), entrega.getHeading())
                .build();

        pathToColetaCima_fromEntrega = follower.pathBuilder()
                .addPath(new BezierLine(new Point(entrega), new Point(Coletacima)))
                .setLinearHeadingInterpolation(entrega.getHeading(), Coletacima.getHeading())
                .build();

        pathToEntrega_fromColetaCima = follower.pathBuilder()
                .addPath(new BezierLine(new Point(Coletacima), new Point(entrega)))
                .setLinearHeadingInterpolation(Coletacima.getHeading(), entrega.getHeading())
                .build();

        pathToComeco_fromEntrega = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(entrega),
                        new Point(voltaEntrga),
                        new Point(Começo)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }
    private void waitForPathToFinish() {
        while (opModeIsActive() &&!isStopRequested() && follower.isBusy()) {
            follower.update();
            follower.telemetryDebug(telemetryA);
            telemetryA.update();
            idle(); // Cede tempo de CPU para outros processos do robô
        }
    }
}