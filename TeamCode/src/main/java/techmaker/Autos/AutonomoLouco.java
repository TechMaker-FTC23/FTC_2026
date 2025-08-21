package techmaker.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
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

@Autonomous(name = "Autonomo Louco")
public class AutonomoLouco extends LinearOpMode {

    // Enum para a FSM, agora com estados granulares para os mecanismos.
    private enum AutoState {
        START,

        // Ciclo Preload
        DRIVE_TO_BASKET_PRELOAD,
        SCORE_SEQUENCE_START,
        SCORE_SEQUENCE_RAISE_ELEVATOR,
        SCORE_SEQUENCE_EXTEND_ARM,
        SCORE_SEQUENCE_OPEN_CLAW,
        SCORE_SEQUENCE_RETRACT,

        // Ciclo 1
        DRIVE_TO_SPIKE_C,
        RELOCALIZE_AT_C,
        COLLECT_SEQUENCE_START,
        COLLECT_SEQUENCE_WAIT,
        DRIVE_TO_BASKET_CYCLE_1,
        // Reutiliza a sequência de pontuação...

        IDLE
    }

    private AutoState currentState = AutoState.START;

    // Hardware e Bibliotecas
    private Follower follower;
    private Limelight3A limelight;
    private ClawSubsystem claw;
    private ElevatorSubsystem elevator;
    private IntakeSubsystem intake;
    private Telemetry telemetryA;
    private ElapsedTime stateTimer = new ElapsedTime();

    // Poses e Caminhos
    private final Pose startPose = new Pose(6.15, 62.44, Math.toRadians(180));
    private final Pose spikeMarkCPose = new Pose(52.83, 54.25, Math.toRadians(268.48));
    private final Pose basketPose = new Pose(51.65, 58.23, Math.toRadians(221.45));
    private PathChain pathToBasketPreload, pathToSpikeC, pathFromSpikeCToBasket;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();
        buildPaths();

        telemetryA.addData("Status", "Autônomo de Competição Pronto.");
        telemetryA.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive() &&!isStopRequested()) {
            // --- ATUALIZAÇÕES CONTÍNUAS ---
            follower.update();
            elevator.update(telemetryA);

            // --- LÓGICA DA FSM ---
            switch (currentState) {
                case START:
                    follower.followPath(pathToBasketPreload);
                    currentState = AutoState.DRIVE_TO_BASKET_PRELOAD;
                    break;

                case DRIVE_TO_BASKET_PRELOAD:
                    if (!follower.isBusy()) {
                        currentState = AutoState.SCORE_SEQUENCE_START;
                    }
                    break;

                // --- Sequência de Pontuação (substitui a thread) ---
                case SCORE_SEQUENCE_START:
                    elevator.goToPositionPID(ElevatorSubsystem.ELEVATOR_PRESET_HIGH);
                    currentState = AutoState.SCORE_SEQUENCE_RAISE_ELEVATOR;
                    break;

                case SCORE_SEQUENCE_RAISE_ELEVATOR:
                    // Garante que a garra fique fechada enquanto o elevador sobe
                    claw.setClawOpen(false);
                    if (elevator.atTargetPosition(20)) {
                        claw.setWristPosition(ClawSubsystem.WRIST_LEFT_SCORE_CLAW, ClawSubsystem.WRIST_RIGHT_SCORE_CLAW);
                        claw.setArmPosition(ClawSubsystem.ARM_LEFT_SCORE_CLAW, ClawSubsystem.ARM_RIGHT_SCORE_CLAW);
                        stateTimer.reset();
                        currentState = AutoState.SCORE_SEQUENCE_EXTEND_ARM;
                    }
                    break;

                case SCORE_SEQUENCE_EXTEND_ARM:
                    if (stateTimer.seconds() > 1.0) { // Espera o braço estabilizar
                        claw.setClawOpen(true);
                        stateTimer.reset();
                        currentState = AutoState.SCORE_SEQUENCE_OPEN_CLAW;
                    }
                    break;

                case SCORE_SEQUENCE_OPEN_CLAW:
                    if (stateTimer.seconds() > 0.5) { // Espera o pixel cair
                        claw.setArmPosition(ClawSubsystem.ARM_LEFT_TRAVEL_CLAW, ClawSubsystem.ARM_RIGHT_TRAVEL_CLAW);
                        elevator.goToPositionPID(ElevatorSubsystem.ELEVATOR_PRESET_GROUND);
                        currentState = AutoState.SCORE_SEQUENCE_RETRACT;
                    }
                    break;

                case SCORE_SEQUENCE_RETRACT:
                    // Espera o elevador descer para começar o próximo movimento
                    if (elevator.atTargetPosition(20)) {
                        follower.followPath(pathToSpikeC);
                        currentState = AutoState.DRIVE_TO_SPIKE_C;
                    }
                    break;

                // --- Ciclo 1 ---
                case DRIVE_TO_SPIKE_C:
                    if (!follower.isBusy()) {
                        currentState = AutoState.RELOCALIZE_AT_C;
                    }
                    break;

                case RELOCALIZE_AT_C:
                    // ÚNICO LUGAR SEGURO PARA ATUALIZAR A POSE
                    updatePoseFromLimelight();
                    currentState = AutoState.COLLECT_SEQUENCE_START;
                    break;

                case COLLECT_SEQUENCE_START:
                    intake.wrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MAX, IntakeSubsystem.RIGHT_INTAKE_WRIST_MAX);
                    intake.sliderMax();
                    intake.startIntake();
                    stateTimer.reset();
                    currentState = AutoState.COLLECT_SEQUENCE_WAIT;
                    break;

                case COLLECT_SEQUENCE_WAIT:
                    // Espera o pixel ser detectado ou o tempo acabar
                    if (intake.isPixelDetected()

                            | stateTimer.seconds() > 2.0) {
                        intake.stopIntake();
                        intake.wrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MIN, IntakeSubsystem.RIGHT_INTAKE_WRIST_MIN);
                        intake.sliderMin();
                        follower.followPath(pathFromSpikeCToBasket);
                        currentState = AutoState.DRIVE_TO_BASKET_CYCLE_1;
                    }
                    break;

                case DRIVE_TO_BASKET_CYCLE_1:
                    if (!follower.isBusy()) {
                        // Reinicia a sequência de pontuação
                        currentState = AutoState.SCORE_SEQUENCE_START;
                    }
                    break;

                case IDLE:
                    // Fim do autônomo.
                    break;
            }
            updateTelemetry();
        }
    }

    // --- MÉTODOS DE INICIALIZAÇÃO E AUXILIARES ---
    // (O conteúdo dos métodos abaixo permanece o mesmo do código anterior)

    private void initializeHardware() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        claw = new ClawSubsystem(hardwareMap);
        elevator = new ElevatorSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap, false);
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        follower.setPose(startPose);
        limelight.pipelineSwitch(0);
        limelight.start();

        claw.setState(ClawSubsystem.ClawState.TRAVEL);
        claw.setClawOpen(false);
        claw.setArmPosition(ClawSubsystem.ARM_LEFT_TRAVEL_CLAW, ClawSubsystem.ARM_RIGHT_TRAVEL_CLAW);
        intake.wrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MIN, IntakeSubsystem.RIGHT_INTAKE_WRIST_MIN);
        intake.sliderMin();
    }

    private void buildPaths() {
        pathToBasketPreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(basketPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), basketPose.getHeading())
                .build();

        pathToSpikeC = follower.pathBuilder()
                .addPath(new BezierLine(new Point(basketPose), new Point(spikeMarkCPose)))
                .setLinearHeadingInterpolation(basketPose.getHeading(), spikeMarkCPose.getHeading())
                .build();

        pathFromSpikeCToBasket = follower.pathBuilder()
                .addPath(new BezierLine(new Point(spikeMarkCPose), new Point(basketPose)))
                .setLinearHeadingInterpolation(spikeMarkCPose.getHeading(), basketPose.getHeading())
                .build();
    }

    private void updatePoseFromLimelight() {
        double currentYawDegrees = Math.toDegrees(follower.getPose().getHeading());
        limelight.updateRobotOrientation(currentYawDegrees);

        LLResult result = limelight.getLatestResult();
        if (result == null ||!result.isValid()) {
            return;
        }

        Pose3D botpose = result.getBotpose_MT2();
        if (botpose!= null) {
            Pose visionPose = new Pose(
                    botpose.getPosition().x * 39.37, // metros para polegadas
                    botpose.getPosition().y * 39.37, // metros para polegadas
                    botpose.getOrientation().getYaw(AngleUnit.RADIANS)
            );
            follower.setPose(visionPose);
        }
    }

    private void updateTelemetry() {
        telemetryA.addData("Current State", currentState.toString());
        telemetryA.addData("Robot Pose", follower.getPose().toString());
        telemetryA.update();
    }
}