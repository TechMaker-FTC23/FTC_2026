package techmaker.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

import techmaker.constants.Constants;
import techmaker.constants.FConstants;
import techmaker.constants.LConstants;
import techmaker.subsystems.ClawSubsystem;
import techmaker.subsystems.ElevatorSubsystem;
import techmaker.subsystems.IntakeSubsystem;
import techmaker.util.DataStorage;
import techmaker.constants.Constants.Intake.SampleColor;

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
        SCORE_SEQUENCE_RETRACT_C,

        // Ciclo 1
        DRIVE_TO_SPIKE_C,
        RELOCALIZE_AT_C,
        COLLECT_SEQUENCE_START,
        COLLECT_SEQUENCE_WAIT,
        DRIVE_TO_BASKET_CYCLE_1,
        DRIVE_TO_BARGE,
        // Reutiliza a sequência de pontuação...

        IDLE
    }

    private AutoState currentState = AutoState.START;
    private int actualSample = 0;

    // Hardware e Bibliotecas
    private Follower follower;
    private Limelight3A limelight;
    private ClawSubsystem claw;
    private ElevatorSubsystem elevator;
    private IntakeSubsystem intake;
    private Telemetry telemetryA;
    private ElapsedTime stateTimer = new ElapsedTime();

    // Poses e Caminhos
    private final Pose startPose = new Pose(25.221, 62.44, Math.toRadians(180));
    private final Pose spikeMarkCPose = new Pose(53.770, 52.593, Math.toRadians(267.57));
    private final Pose SpikeMarkDPose = new Pose(44.319, 53.6127, Math.toRadians(268.13));
    private final Pose SpikeMarkEPose = new Pose(52.53, 52.859, Math.toRadians(294.33));
    private final Pose basketPose = new Pose(55.8, 58.4, Math.toRadians(230));
    private final Pose bargePose = new Pose(32.40432859405758, -3.973167599655512, Math.toRadians(190.19982702485984));
    private PathChain pathToBasketPreload, pathToSpikeC, pathFromSpikeCToBasket, pathToSpikeE, pathFromSpikeEToBasket, pathToSpikeD, pathFromSpikeDToBasket, pathToBarge;


    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();
        DataStorage.allianceColor = SampleColor.Red;
        telemetryA.addData("Status", "Autônomo de Competição Pronto.");
        telemetryA.addData("Pose",follower.getPose());
        telemetryA.addData("Erro na pose inicial",!startPose.roughlyEquals(follower.getPose(),5));
        telemetryA.addData("Aliança",DataStorage.allianceColor);
        telemetryA.update();

        waitForStart();
        if (isStopRequested()) return;
        if (DataStorage.allianceColor == SampleColor.Red) {
            buildPathsRed();
            intake.setIsRedAlliance(true);
        }
        else {
            buildPathsBlue();
            intake.setIsRedAlliance(false);
        }

        while (opModeIsActive() &&!isStopRequested()) {
            // --- ATUALIZAÇÕES CONTÍNUAS ---
            follower.update();
            elevator.update(telemetryA);
            DataStorage.robotPose = follower.getPose();

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
                        claw.setArmPosition(ClawSubsystem.ARM_LEFT_INTAKE_CLAW, ClawSubsystem.ARM_RIGHT_INTAKE_CLAW);
                        claw.setClawOpen(false);
                    if (stateTimer.seconds() > 0.2) {
                        intake.reverseIntake();
                    }

                    if (stateTimer.seconds() > 0.6) {
                        elevator.goToPositionPID(ElevatorSubsystem.ELEVATOR_PRESET_HIGH);
                        currentState = AutoState.SCORE_SEQUENCE_RAISE_ELEVATOR;
                    }

                    break;

                case SCORE_SEQUENCE_RAISE_ELEVATOR:
                    if (elevator.atTargetPosition(20)) {
                        claw.setWristPosition(ClawSubsystem.WRIST_LEFT_SCORE_CLAW, ClawSubsystem.WRIST_RIGHT_SCORE_CLAW);
                        claw.setArmPosition(ClawSubsystem.ARM_LEFT_SCORE_CLAW, ClawSubsystem.ARM_RIGHT_SCORE_CLAW);
                        stateTimer.reset();
                        currentState = AutoState.SCORE_SEQUENCE_EXTEND_ARM;
                    }
                    break;

                case SCORE_SEQUENCE_EXTEND_ARM:
                    if (stateTimer.seconds() > 1.0) {
                        claw.setClawOpen(true);
                        stateTimer.reset();
                        currentState = AutoState.SCORE_SEQUENCE_OPEN_CLAW;
                    }
                    break;

                case SCORE_SEQUENCE_OPEN_CLAW:
                    if (stateTimer.seconds() > 0.5) {
                        claw.setArmPosition(ClawSubsystem.ARM_LEFT_TRAVEL_CLAW, ClawSubsystem.ARM_RIGHT_TRAVEL_CLAW);
                        claw.setWristPosition(ClawSubsystem.WRIST_LEFT_TRAVEL_CLAW, ClawSubsystem.WRIST_RIGHT_TRAVEL_CLAW);
                        elevator.goToPositionPID(ElevatorSubsystem.ELEVATOR_PRESET_GROUND);
                        currentState = AutoState.SCORE_SEQUENCE_RETRACT_C;
                        stateTimer.reset();
                        if(actualSample==3){
                            follower.followPath(pathToSpikeE);
                            currentState = AutoState.DRIVE_TO_BARGE;
                        }
                    }
                    break;

                case SCORE_SEQUENCE_RETRACT_C:
                    if (elevator.atTargetPosition(20)) {
                        intake.wrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MAX, IntakeSubsystem.RIGHT_INTAKE_WRIST_MAX);
                        intake.sliderMax();
                        intake.startIntake();
                     if (stateTimer.seconds() > 1.5){
                         if(actualSample==0) {
                             follower.followPath(pathToSpikeC);
                         }
                         if(actualSample==1){
                             follower.followPath(pathToSpikeD);
                         }
                         if(actualSample==2){
                             follower.followPath(pathToSpikeE);
                         }
                         currentState = AutoState.DRIVE_TO_SPIKE_C;

                         stateTimer.reset();

                     }


                    }
                    break;

                // --- Ciclo 1 ---
                case DRIVE_TO_SPIKE_C:
                    if (!follower.isBusy()) {
                        currentState = AutoState.RELOCALIZE_AT_C;
                    }
                    break;

                case RELOCALIZE_AT_C:
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
                    if (intake.isPixelDetected() || stateTimer.seconds() > 1.0) {
                        intake.stopIntake();
                        intake.wrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MIN, IntakeSubsystem.RIGHT_INTAKE_WRIST_MIN);
                        intake.sliderMin();
                        if(actualSample==0) {
                            follower.followPath(pathFromSpikeCToBasket);

                        }
                        if(actualSample==1){
                            follower.followPath(pathFromSpikeDToBasket);
                        }
                        if(actualSample==2){
                            follower.followPath(pathFromSpikeDToBasket);

                        }
                        follower.followPath(pathFromSpikeCToBasket);
                        currentState = AutoState.DRIVE_TO_BASKET_CYCLE_1;
                    }
                    break;

                case DRIVE_TO_BASKET_CYCLE_1:
                    if (!follower.isBusy()) {
                        stateTimer.reset();
                        currentState = AutoState.SCORE_SEQUENCE_START;
                        actualSample++;
                    }
                    break;
                case DRIVE_TO_BARGE:
                    if (!follower.isBusy()) {
                        currentState = AutoState.IDLE;
                    }
                    break;

                case IDLE:
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
        follower.update();

        limelight.pipelineSwitch(0);
        limelight.start();
        limelight.updateRobotOrientation(Math.toDegrees(startPose.getHeading()));
        updatePoseFromLimelight();

        claw.setState(ClawSubsystem.ClawState.TRAVEL);
        claw.setClawOpen(false);
        claw.setArmPosition(ClawSubsystem.ARM_LEFT_TRAVEL_CLAW, ClawSubsystem.ARM_RIGHT_TRAVEL_CLAW);
        intake.wrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MIN, IntakeSubsystem.RIGHT_INTAKE_WRIST_MIN);
        intake.sliderMin();
    }

    private void buildPathsBlue() {
        pathToBasketPreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(follower.getPose()), new Point(basketPose)))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), basketPose.getHeading())
                .build();

        pathToSpikeC = follower.pathBuilder()
                .addPath(new BezierLine(new Point(basketPose), new Point(spikeMarkCPose)))
                .setConstantHeadingInterpolation(spikeMarkCPose.getHeading())
                .build();

        pathFromSpikeCToBasket = follower.pathBuilder()
                .addPath(new BezierLine(new Point(spikeMarkCPose), new Point(basketPose)))
                .setConstantHeadingInterpolation(basketPose.getHeading())
                .build();

        pathToSpikeD = follower.pathBuilder()
                .addPath(new BezierLine(new Point(basketPose), new Point(SpikeMarkDPose)))
                .setConstantHeadingInterpolation(SpikeMarkDPose.getHeading())
                .build();

        pathFromSpikeDToBasket  = follower.pathBuilder()
                .addPath(new BezierLine(new Point(SpikeMarkDPose), new Point(basketPose)))
                .setConstantHeadingInterpolation(basketPose.getHeading())
                .build();

        pathToSpikeE = follower.pathBuilder()
                .addPath(new BezierLine(new Point(basketPose), new Point(SpikeMarkEPose)))
                .setConstantHeadingInterpolation(SpikeMarkEPose.getHeading())
                .build();

        pathFromSpikeEToBasket = follower.pathBuilder()
                .addPath(new BezierLine(new Point(SpikeMarkEPose), new Point(basketPose)))
                .setConstantHeadingInterpolation(basketPose.getHeading())
                .build();
        pathToBarge = follower.pathBuilder()
                .addPath(new BezierLine(new Point(basketPose), new Point(bargePose)))
                .setConstantHeadingInterpolation(bargePose.getHeading())
                .build();
    }
    private void buildPathsRed() {
        Pose poseInit = follower.getPose();
        Pose poseFinish = invertPose(basketPose);
        pathToBasketPreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(poseInit), new Point( poseFinish)))
                .setLinearHeadingInterpolation(poseInit.getHeading(), poseFinish.getHeading())
                .build();

        poseInit = invertPose(basketPose);
        poseFinish = invertPose(spikeMarkCPose);
        pathToSpikeC = follower.pathBuilder()
                .addPath(new BezierLine(new Point(poseInit), new Point( poseFinish)))
                .setConstantHeadingInterpolation(poseFinish.getHeading())
                .build();

        pathFromSpikeCToBasket = follower.pathBuilder()
                .addPath(new BezierLine(new Point(poseFinish), new Point(poseInit)))
                .setConstantHeadingInterpolation(poseInit.getHeading())
                .build();

        poseFinish = invertPose(SpikeMarkDPose);
        pathToSpikeD = follower.pathBuilder()
                .addPath(new BezierLine(new Point(poseInit), new Point( poseFinish)))
                .setConstantHeadingInterpolation(poseFinish.getHeading())
                .build();

       pathFromSpikeDToBasket  = follower.pathBuilder()
               .addPath(new BezierLine(new Point(poseFinish), new Point(poseInit)))
               .setConstantHeadingInterpolation(poseInit.getHeading())
               .build();


        poseFinish = invertPose(SpikeMarkEPose);
        pathToSpikeE = follower.pathBuilder()
                .addPath(new BezierLine(new Point(poseInit), new Point( poseFinish)))
                .setConstantHeadingInterpolation(poseFinish.getHeading())
                .build();

        pathFromSpikeEToBasket = follower.pathBuilder()
                .addPath(new BezierLine(new Point(poseFinish), new Point(poseInit)))
                .setConstantHeadingInterpolation(poseInit.getHeading())
                .build();

        poseFinish = invertPose(bargePose);
        pathToBarge = follower.pathBuilder()
                .addPath(new BezierLine(new Point(poseInit), new Point( poseFinish)))
                .setConstantHeadingInterpolation(poseFinish.getHeading())
                .build();
    }
    public SampleColor getRobotAllianceViaApriltag(){
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    if(fr.getFiducialId()==13){
                        return SampleColor.Blue;
                    }
                    if(fr.getFiducialId()==16){
                        return SampleColor.Red;
                    }

                }
            }

        }
        return SampleColor.Idle;

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
            follower.update();
        }
    }
    public Pose invertPose(Pose pose){
        return new Pose(-pose.getX(),-pose.getY(),pose.getHeading()-Math.PI);
    }

    private void updateTelemetry() {
        telemetryA.addData("Current State", currentState.toString());
        telemetryA.addData("Robot Pose", follower.getPose().toString());
        telemetryA.addData("Actual Sample",actualSample);
        telemetryA.update();
    }
}