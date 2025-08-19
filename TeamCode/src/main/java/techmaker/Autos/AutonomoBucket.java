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

@Autonomous(name = "Autônomo Basket")
public class AutonomoBucket extends LinearOpMode {

    // Enum para as posições detetadas do Signal Cone (fundamental para um autônomo real)
    public enum SignalPosition { LEFT, CENTER, RIGHT }

    private Telemetry telemetryA;
    private Follower follower;
    private ClawSubsystem claw;
    private ElevatorSubsystem elevator;
    private IntakeSubsystem intake;
    // private VisionSubsystem vision; // Descomentar quando a visão for implementada

    // --- Poses de Jogo Otimizadas para Precisão ---
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));

    // Posição de entrega
    private final Pose entrega = new Pose(-20, -20, Math.toRadians(135));
    // Ponto de controlo para o arco largo
    private final Pose viaPointEntrega = new Pose(40, 20, Math.toRadians(90));

    // Posição de APROXIMAÇÃO ao Chute (pilha de Drones). O robô alinha-se aqui.
    private final Pose ChuteApproach = new Pose(80, 90, Math.toRadians(180));
    // Posição final de coleta no Chute.
    private final Pose ChuteCollection = new Pose(100, 90, Math.toRadians(180));

    // Posição para a coleta do "ColetaMeio"
    private final Pose ColetaMeio = new Pose(100, -30, Math.toRadians(-90));

    // Posição final de estacionamento
    private final Pose Parking = new Pose(100, -20, Math.toRadians(90));

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        // Durante o init, a visão determina a posição do Signal Cone
        // SignalPosition detectedPosition = vision.getDetection();
        SignalPosition detectedPosition = SignalPosition.CENTER; // Valor padrão para testes

        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {

            // --- ROTINA PRINCIPAL ---

            // PASSO 1: Pontuar o Drone pré-carregado com um arco largo.
            scorePreloadWithArc();

            // PASSO 2: Primeiro ciclo de coleta (do Chute).
            runCollectionCycle(ChuteApproach, ChuteCollection, ElevatorSubsystem.ELEVATOR_PRESET_MEDIUM, 5); // Coleta 5 Drones

            // PASSO 3: Segundo ciclo de coleta (do Chute).
            runCollectionCycle(ChuteApproach, ChuteCollection, ElevatorSubsystem.ELEVATOR_PRESET_MEDIUM, 3); // Coleta 3 Drones

            // PASSO 4: Terceiro ciclo de coleta (do Chute).
            // runCollectionCycle(ChuteApproach, ChuteCollection, ElevatorSubsystem.ELEVATOR_PRESET_MEDIUM, 1); // Coleta o último Drone

            // PASSO 5: Ciclo de coleta final (do ColetaMeio).
            // runSingleCollectionCycle(ColetaMeio, ElevatorSubsystem.ELEVATOR_PRESET_HIGH);

            // PASSO 6: Estacionar.
            park();
        }
    }

    private void initialize() {
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        claw = new ClawSubsystem(hardwareMap);
        elevator = new ElevatorSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap, false);
        // vision = new VisionSubsystem(hardwareMap);

        follower.setStartingPose(startPose);
        claw.setState(ClawSubsystem.ClawState.TRAVEL);
        claw.setClawOpen(false);
        elevator.goToPositionPID(ElevatorSubsystem.ELEVATOR_PRESET_GROUND);
        intake.sliderMin();

        telemetryA.addLine("Autônomo de Precisão Pronto.");
        telemetryA.update();
    }

    private void scorePreloadWithArc() {
        telemetryA.addLine("Passo 1: A pontuar o pré-carregado...");
        telemetryA.update();

        elevator.goToPositionPID(ElevatorSubsystem.ELEVATOR_PRESET_HIGH);
        claw.setState(ClawSubsystem.ClawState.SCORE);

        PathChain pathToEntrega = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(follower.getPose()), new Point(viaPointEntrega), new Point(entrega)))
                .build();
        follower.followPath(pathToEntrega, true);

        waitForPathToFinish();
        waitForElevator(ElevatorSubsystem.ELEVATOR_PRESET_HIGH);

        claw.setClawOpen(true);
        sleep(300);
    }

    private void runCollectionCycle(Pose approachPose, Pose collectionPose, int scoreHeight, int dronesInStack) {
        // --- IR PARA A POSIÇÃO DE APROXIMAÇÃO ---
        telemetryA.addLine("Ciclo: A ir para a aproximação do Chute...");
        telemetryA.update();

        claw.setState(ClawSubsystem.ClawState.TRAVEL);
        elevator.goToPositionPID(ElevatorSubsystem.ELEVATOR_PRESET_GROUND);

        PathChain pathToApproach = follower.pathBuilder()
                .addPath(new BezierLine(new Point(follower.getPose()), new Point(approachPose)))
                .build();
        follower.followPath(pathToApproach, true);
        waitForPathToFinish();
        waitForElevator(ElevatorSubsystem.ELEVATOR_PRESET_GROUND);

        // --- PASSO CRÍTICO DE PRECISÃO: RECALIBRAR POSIÇÃO ---
        // Pose correctedPose = vision.getPoseFromAprilTags();
        // if (correctedPose != null) {
        //     follower.setPose(correctedPose);
        //     telemetryA.addLine("Posição recalibrada com AprilTags!");
        //     telemetryA.update();
        // }

        // --- COLETAR ---
        collectFromStack(collectionPose, dronesInStack);

        // --- PONTUAR O DRONE COLETADO ---
        scoreDrone(scoreHeight);
    }

    private void collectFromStack(Pose collectionPose, int dronesInStack) {
        telemetryA.addLine("Ciclo: A iniciar sequência de coleta precisa...");
        telemetryA.update();

        // Ajusta a altura do intake para o topo da pilha
        // Esta lógica assume que você implementou um método para controlar a altura do intake
        // intake.setHeight(IntakeSubsystem.StackHeight.fromDrones(dronesInStack));

        intake.sliderMax();
        sleep(250);
        intake.wrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MAX, IntakeSubsystem.RIGHT_INTAKE_WRIST_MAX);

        // Avança lentamente para dentro da pilha
        PathChain pathToStack = follower.pathBuilder()
                .addPath(new BezierLine(new Point(follower.getPose()), new Point(collectionPose)))
                .build();
        follower.followPath(pathToStack, true);

        intake.startAutomaticCapture();

        // Espera pela captura enquanto o robô se move
        ElapsedTime captureTimer = new ElapsedTime();
        while(opModeIsActive() && !intake.isCaptureComplete() && captureTimer.seconds() < 2.0) {
            intake.updateAutomaticCapture();
            follower.update(); // Continua a seguir o caminho
        }
        waitForPathToFinish(); // Garante que o movimento terminou

        claw.setClawOpen(false);
        sleep(250);

        // Recua da pilha para uma posição segura
        PathChain pathBackFromStack = follower.pathBuilder()
                .addPath(new BezierLine(new Point(follower.getPose()), new Point(ChuteApproach)))
                .build();
        follower.followPath(pathBackFromStack, true);

        // Retrai o intake enquanto recua
        intake.wrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MIN, IntakeSubsystem.RIGHT_INTAKE_WRIST_MIN);
        intake.sliderMin();
        waitForPathToFinish();
    }

    private void scoreDrone(int scoreHeight) {
        telemetryA.addLine("Ciclo: A retornar para pontuar...");
        telemetryA.update();

        elevator.goToPositionPID(scoreHeight);
        claw.setState(ClawSubsystem.ClawState.SCORE);

        PathChain pathToEntrega = follower.pathBuilder()
                .addPath(new BezierLine(new Point(follower.getPose()), new Point(entrega)))
                .build();
        follower.followPath(pathToEntrega, true);
        waitForPathToFinish();
        waitForElevator(scoreHeight);

        claw.setClawOpen(true);
        sleep(300);
    }

    private void park() {
        telemetryA.addLine("A estacionar...");
        telemetryA.update();

        claw.setState(ClawSubsystem.ClawState.TRAVEL);
        elevator.goToPositionPID(ElevatorSubsystem.ELEVATOR_PRESET_GROUND);

        PathChain pathToParking = follower.pathBuilder()
                .addPath(new BezierLine(new Point(follower.getPose()), new Point(Parking)))
                .build();
        follower.followPath(pathToParking, true);
        waitForPathToFinish();
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
