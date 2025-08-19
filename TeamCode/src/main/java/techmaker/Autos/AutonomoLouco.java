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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import techmaker.constants.FConstants;
import techmaker.constants.LConstants;
import techmaker.subsystems.ClawSubsystem;
import techmaker.subsystems.ElevatorSubsystem;
import techmaker.subsystems.IntakeSubsystem;

@Autonomous(name = "Autônomo de Campeonato (Coleta Inteligente)")
public class AutonomoLouco extends LinearOpMode {

    private Telemetry telemetryA;
    private Follower follower;
    private ClawSubsystem claw;
    private ElevatorSubsystem elevator;
    private IntakeSubsystem intake;

    // --- Poses de Jogo Otimizadas (Ajustar com precisão no campo!) ---
    private final double INTAKE_EXTENSION_DISTANCE = 20.0; // Distância que o slider estende (em cm)
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));

    // Poses dos Drones nos Spike Marks (baseado no manual: 3.5" da parede, 10" entre eles)
    private final Pose spikeMarkCenter = new Pose(88.9, 0, Math.toRadians(0));
    private final Pose spikeMarkLeft = new Pose(88.9, 25.4, Math.toRadians(0));
    private final Pose spikeMarkRight = new Pose(88.9, -25.4, Math.toRadians(0));

    // Poses de APROXIMAÇÃO para cada Spike Mark (recuadas pela distância do intake)
    private final Pose spikeMarkCenterApproach = new Pose(spikeMarkCenter.getX() - INTAKE_EXTENSION_DISTANCE, spikeMarkCenter.getY(), spikeMarkCenter.getHeading());
    private final Pose spikeMarkLeftApproach = new Pose(spikeMarkLeft.getX() - INTAKE_EXTENSION_DISTANCE, spikeMarkLeft.getY(), spikeMarkLeft.getHeading());
    private final Pose spikeMarkRightApproach = new Pose(spikeMarkRight.getX() - INTAKE_EXTENSION_DISTANCE, spikeMarkRight.getY(), spikeMarkRight.getHeading());

    // Posição de lançamento nos Baskets
    private final Pose basketLaunchPose = new Pose(90, -85, Math.toRadians(90));
    // Ponto de controlo para o arco largo
    private final Pose viaPointEntrega = new Pose(40, 20, Math.toRadians(90));

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {
            // --- ROTINA PRINCIPAL ---

            scorePreload();
            runSpikeMarkCycle(spikeMarkCenterApproach, spikeMarkCenter);
            runSpikeMarkCycle(spikeMarkLeftApproach, spikeMarkLeft);
            runSpikeMarkCycle(spikeMarkRightApproach, spikeMarkRight);

            telemetryA.addLine("Rotina de 4 Drones concluída!");
            telemetryA.update();
        }
    }

    private void initialize() {
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        claw = new ClawSubsystem(hardwareMap);
        elevator = new ElevatorSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap, false);

        follower.setStartingPose(startPose);
        claw.setState(ClawSubsystem.ClawState.TRAVEL);
        claw.setClawOpen(false);
        elevator.goToPositionPID(ElevatorSubsystem.ELEVATOR_PRESET_GROUND);
        intake.sliderMin();

        telemetryA.addLine("Autônomo de Precisão Pronto.");
        telemetryA.update();
    }

    private void scorePreload() {
        telemetryA.addLine("Passo 1: A pontuar o pré-carregado...");
        telemetryA.update();

        elevator.goToPositionPID(ElevatorSubsystem.ELEVATOR_PRESET_HIGH);
        claw.setState(ClawSubsystem.ClawState.SCORE);

        PathChain pathToLaunch = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(follower.getPose()), new Point(viaPointEntrega), new Point(basketLaunchPose)))
                .build();
        follower.followPath(pathToLaunch, true);

        waitForPathToFinish();
        waitForElevator(ElevatorSubsystem.ELEVATOR_PRESET_HIGH);

        claw.setClawOpen(true);
        sleep(200); // Sleep reduzido
    }

    private void runSpikeMarkCycle(Pose approachPose, Pose collectionPose) {
        telemetryA.addLine("Ciclo: A ir para a aproximação em " + approachPose);
        telemetryA.update();

        claw.setState(ClawSubsystem.ClawState.TRAVEL);
        elevator.goToPositionPID(ElevatorSubsystem.ELEVATOR_PRESET_GROUND);

        PathChain pathToApproach = follower.pathBuilder()
                .addPath(new BezierLine(new Point(follower.getPose()), new Point(approachPose)))
                .build();
        follower.followPath(pathToApproach, true);
        waitForPathToFinish();
        waitForElevator(ElevatorSubsystem.ELEVATOR_PRESET_GROUND);

        // Executa a manobra de coleta inteligente e verifica se foi bem-sucedida
        boolean collectionSuccess = collectFromSpike(collectionPose);

        if (collectionSuccess) {
            scoreCollectedDrone();
        } else {
            telemetryA.addLine("COLETA FALHOU! A abortar este ciclo.");
            telemetryA.update();
            // Retrai o intake para segurança antes de continuar
            intake.wrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MIN, IntakeSubsystem.RIGHT_INTAKE_WRIST_MIN);
            intake.sliderMin();
            sleep(250);
        }
    }

    private boolean collectFromSpike(Pose targetSpikePose) {
        telemetryA.addLine("A executar manobra de coleta inteligente...");
        telemetryA.update();

        intake.wrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MAX, IntakeSubsystem.RIGHT_INTAKE_WRIST_MAX);
        intake.sliderMax();
        sleep(250); // Sleep reduzido para a extensão do slider

        intake.resetCaptureState();
        intake.startAutomaticCapture();

        // Inicia um movimento lento para a frente para "procurar" o Drone
        PathChain searchPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(follower.getPose()), new Point(targetSpikePose)))
                .build();
        follower.followPath(searchPath, true);

        // Loop de espera ativa: espera que a captura aconteça OU que o movimento termine
        while(opModeIsActive() && !isStopRequested() && follower.isBusy() && !intake.isCaptureComplete()) {
            follower.update();
            intake.updateAutomaticCapture();
        }

        // Verifica o resultado
        if (intake.isCaptureComplete()) {
            // CORREÇÃO: Para o movimento do robô imediatamente enviando vetores de movimento nulos.
            follower.setTeleOpMovementVectors(0, 0, 0, false);
            telemetryA.addLine("Drone DETETADO! A segurar...");
            telemetryA.update();

            claw.setClawOpen(false);
            sleep(250);
            intake.sliderMin();
            sleep(250);
            intake.wrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MIN, IntakeSubsystem.RIGHT_INTAKE_WRIST_MIN);
            return true; // Sucesso!
        } else {
            // Se chegou aqui, o caminho terminou mas o Drone não foi detetado
            telemetryA.addLine("Busca concluída, Drone não encontrado.");
            telemetryA.update();
            intake.sliderMin(); // Retrai por segurança
            return false; // Falha!
        }
    }

    private void scoreCollectedDrone() {
        telemetryA.addLine("Ciclo: A retornar para pontuar...");
        telemetryA.update();

        elevator.goToPositionPID(ElevatorSubsystem.ELEVATOR_PRESET_MEDIUM);
        claw.setState(ClawSubsystem.ClawState.SCORE);

        PathChain pathToLaunch = follower.pathBuilder()
                .addPath(new BezierLine(new Point(follower.getPose()), new Point(basketLaunchPose)))
                .build();
        follower.followPath(pathToLaunch, true);
        waitForPathToFinish();
        waitForElevator(ElevatorSubsystem.ELEVATOR_PRESET_MEDIUM);

        claw.setClawOpen(true);
        // Otimização: Iniciar a próxima ação imediatamente após abrir a garra
        // sleep(200); // Sleep removido ou muito reduzido
    }

    private void waitForPathToFinish() {
        while (opModeIsActive() && !isStopRequested() && follower.isBusy()) {
            follower.update();
            elevator.update(telemetryA);
            intake.updateAutomaticCapture();
            follower.telemetryDebug(telemetryA);
            telemetryA.update();
        }
    }

    private void waitForElevator(int targetPosition) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && !isStopRequested() && Math.abs(elevator.getCurrentPosition() - targetPosition) > 20 && timer.seconds() < 2.5) {
            elevator.update(telemetryA);
        }
    }
}
