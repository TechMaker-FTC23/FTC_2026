package techmaker;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import techmaker.constants.Constants;
import techmaker.constants.FConstants;
import techmaker.constants.LConstants;
import techmaker.subsystems.ClawSubsystem;
import techmaker.subsystems.ElevatorSubsystem;
import techmaker.subsystems.IntakeSubsystem;
import techmaker.util.DataStorage;
import techmaker.util.StateMachine;

@TeleOp(name = "TeleOp Posições Fixas")
public class TeleopPosiçõesFixas extends OpMode {
    private ClawSubsystem claw;
    private ElevatorSubsystem elevator;
    private IntakeSubsystem intake;
    private Follower follower;
    private Limelight3A limelight;
    private StateMachine state = StateMachine.IDLE;
    private StateMachine stateClawSample = StateMachine.CLAW_SPECIMENT;
    private final Pose startPose = new Pose(0, 0, Math.PI);
    private final Pose BargeUp = new Pose(-8.774839386226624, 53.486424243356296, Math.toRadians(257.56982137500916));
    private final Pose BargeMiddle = new Pose(32.40432859405758, -3.973167599655512, Math.toRadians(190.19982702485984));
    private final Pose Basket = new Pose(51.65671040692668, 58.230110228531004, Math.toRadians(221.45235477987202));
    private final ElapsedTime timer = new ElapsedTime();
    private long timeout = 0;


    @Override
    public void init() {
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        if (DataStorage.robotPose!= null) {
            follower.setStartingPose(DataStorage.robotPose);
        } else {
            follower.setStartingPose(startPose);
        }
        follower.update();
        claw = new ClawSubsystem(hardwareMap);
        elevator = new ElevatorSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap, false);
        intake.setIsRedAlliance(DataStorage.allianceColor == Constants.Intake.SampleColor.Red);


        // MUDANÇA: Define a posição inicial segura da garra com um único comando.
        claw.setState(ClawSubsystem.ClawState.TRAVEL);
        claw.setClawOpen(true); // Começa com a garra aberta
        claw.setArmPosition(ClawSubsystem.ARM_LEFT_TRAVEL_CLAW, ClawSubsystem.ARM_RIGHT_TRAVEL_CLAW);
        intake.wrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MIN, IntakeSubsystem.RIGHT_INTAKE_WRIST_MIN);
        intake.sliderMin();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        telemetry.addData("Status", "TeleOp Principal Inicializado");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        intake.sliderMin();
        intake.update(telemetry);
        telemetry.update();
        if (DataStorage.robotPose!= null) {
            follower.setStartingPose(DataStorage.robotPose);
        }
        follower.update();
        updatePoseFromLimelight();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        timer.reset();
    }


    @Override
    public void loop() {
        follower.update();

        // --- NOVO: Botão de Cancelamento / Pânico ---
        if (gamepad1.dpad_down) {
            follower.breakFollowing();
        }

        // --- LÓGICA DE CONTROLE DO CHASSI: MANUAL VS. AUTOMÁTICO ---
        if (follower.isBusy()) {
            // MODO AUTOMÁTICO: O robô está se movendo para um alvo.
            // Os joysticks de direção são ignorados para não haver interferência.
        } else {
            // MODO MANUAL: O robô está livre.
            double x = gamepad1.left_stick_y;
            double y = -gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;

            if(DataStorage.allianceColor== Constants.Intake.SampleColor.Red){
                x = -x;
                y = -y;
                turn = -turn;
            }

            double heading = follower.getPose().getHeading();

            follower.setTeleOpMovementVectors(y, x, turn, gamepad1.right_bumper);
            updatePoseFromLimelight();

            // --- NOVO: Comandos de Mira Automática (Posições Fixas) ---
            if (gamepad1.cross) {
                Pose targetBasket = (DataStorage.allianceColor == Constants.Intake.SampleColor.Red)? invertPose(Basket) : Basket;
                goToPoint(targetBasket);
            } else if (gamepad1.circle) {
                Pose targetBargeUp = (DataStorage.allianceColor == Constants.Intake.SampleColor.Red)? invertPose(BargeUp) : BargeUp;
                goToPoint(targetBargeUp);
            } else if (gamepad1.square) {
                Pose targetBargeMiddle = (DataStorage.allianceColor == Constants.Intake.SampleColor.Red)? invertPose(BargeMiddle) : BargeMiddle;
                goToPoint(targetBargeMiddle);
            }
        }

        intake.maintainSliderPosition();

        if (gamepad2.triangle && state == StateMachine.IDLE) {
            state = StateMachine.START_INTAKE;
            intake.wrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MAX, IntakeSubsystem.RIGHT_INTAKE_WRIST_MAX);
            intake.startIntake();
            timeout = 200;
            timer.reset();
        } else if (state == StateMachine.INTAKING && (gamepad2.circle || intake.isPixelDetected())) {
            state = StateMachine.INTAKE_DETECTING;
            timeout = 100;
            timer.reset();

        }
        if(gamepad1.dpad_up) {
            if (DataStorage.allianceColor == Constants.Intake.SampleColor.Red) {
                follower.setPose(invertPose(startPose));
            } else {
                follower.setPose(startPose);
            }

            follower.update();
            updatePoseFromLimelight();
        }

        if (gamepad2.right_bumper && stateClawSample == StateMachine.CLAW_SPECIMENT) {
            stateClawSample = StateMachine.CLAW_SAMPLE;
            claw.setArmPosition(ClawSubsystem.ARM_LEFT_INTAKE_CLAW, ClawSubsystem.ARM_RIGHT_INTAKE_CLAW);
            intake.reverseIntake();
            timeout = 100;
            timer.reset();
        } else if (gamepad2.left_bumper && stateClawSample == StateMachine.DELIVERY_SPECIMENT) {
            stateClawSample = StateMachine.CLAW_RETRACT;

            claw.setClawOpen(true);
            timeout = 200;
            timer.reset();
        }

        if (gamepad2.cross && state == StateMachine.IDLE) {
            state = StateMachine.AUTO_CYCLE_START;
            intake.sliderMax();
            timeout = 100;
            timer.reset();
        }

        if (gamepad2.dpad_right) {
            intake.reverseIntake();
        }

        if (state == StateMachine.REVERTING_INTAKE &&!intake.isPixelDetected()) {
            state = StateMachine.RETURNING_INTAKE;
            intake.wrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MIN, IntakeSubsystem.RIGHT_INTAKE_WRIST_MIN);
            intake.stopIntake();
            timeout = 100;
            timer.reset();
        }
        if (timer.milliseconds() > timeout) {
            if (state == StateMachine.START_INTAKE) {
                intake.sliderMax();

                state = StateMachine.INTAKING;
            } else if (state == StateMachine.INTAKE_DETECTING) {
                if (intake.isSampleCorrectAlliance()) {
                    gamepad1.rumble(200);
                    state = StateMachine.RETURNING_INTAKE;
                    intake.wrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MIN, IntakeSubsystem.RIGHT_INTAKE_WRIST_MIN);
                    intake.stopIntake();
                    intake.sliderMin();
                    timeout = 200;
                    timer.reset();
                } else {
                    intake.reverseIntake();
                    state = StateMachine.REVERTING_INTAKE;
                }
            } else if (state == StateMachine.RETURNING_INTAKE) {
                // NOVO: ROTA AUTOMÁTICA PARA A CESTA
                // Quando o timer de 500ms termina, o slider já recuou.
                // Agora, iniciamos o caminho para a cesta.
                Pose targetBasket = (DataStorage.allianceColor == Constants.Intake.SampleColor.Red)? invertPose(Basket) : Basket;
                goToPoint(targetBasket);
                state = StateMachine.IDLE; // A state machine dos mecanismos pode voltar para IDLE.
            } else if (state == StateMachine.AUTO_CYCLE_START) {
                intake.wrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MAX, IntakeSubsystem.RIGHT_INTAKE_WRIST_MAX);
                intake.startIntake();
                state = StateMachine.AUTO_INTAKING;
                timeout = 400;
                timer.reset();
            } else if (state == StateMachine.AUTO_INTAKING) {
                // MUDANÇA: Usa a nova API da garra.
                claw.setClawOpen(false);
                claw.setState(ClawSubsystem.ClawState.INTAKE);
                intake.reverseIntake();
                state = StateMachine.AUTO_GRAB;
                timeout = 200;
                timer.reset();
            } else if (state == StateMachine.AUTO_GRAB) {
                intake.stopIntake();
                // MUDANÇA: Usa a nova API da garra.
                claw.setState(ClawSubsystem.ClawState.SCORE);
                state = StateMachine.AUTO_RAISE;
                timeout = 200;
                timer.reset();
            } else if (state == StateMachine.AUTO_RAISE) {
                state = StateMachine.IDLE;
            }

            if (stateClawSample == StateMachine.CLAW_SAMPLE) {
                claw.setClawOpen(false);

                stateClawSample = StateMachine.CLAW_SCORING;
                timeout = 200;
                timer.reset();
            } else if (stateClawSample == StateMachine.CLAW_SCORING) {
                claw.setClawOpen(false);
                elevator.goToPositionPID(ElevatorSubsystem.ELEVATOR_PRESET_HIGH);
                stateClawSample = StateMachine.DELIVER_SAMPLE;
                timeout = 200;
                timer.reset();
            }else if (stateClawSample == StateMachine.DELIVER_SAMPLE) {
                claw.setState(ClawSubsystem.ClawState.SCORE);
                intake.stopIntake();
                stateClawSample = StateMachine.DELIVERY_SPECIMENT;
            } else if (stateClawSample == StateMachine.CLAW_RETRACT) {
                claw.setState(ClawSubsystem.ClawState.TRAVEL);
                elevator.goToPositionPID(ElevatorSubsystem.ELEVATOR_PRESET_GROUND);
                intake.stopIntake();
                stateClawSample = StateMachine.CLAW_SPECIMENT;
            }
        }

        Pose pose = follower.getPose();
        //claw.update(telemetry);
        elevator.update(telemetry);
        intake.update(telemetry);
        telemetry.addData("Main State", state);
        //telemetry.addData("Claw State", stateClawSample);
        telemetry.addData("Pose", pose);
        telemetry.addData("Mode", follower.isBusy()? "AUTOMATICO" : "MANUAL"); // Telemetria de modo
        telemetry.update();

    }
    public void tranferSample(){

        intake.reverseIntake();
        double time = getRuntime()+0.1;
        while(getRuntime()<time){

        }
        intake.stopIntake();
        claw.setClawOpen(false);


    }

    /**
     * NOVO: Gera e segue um caminho da posição atual até um ponto de destino.
     * @param targetPose O ponto final para onde o robô deve ir.
     */
    private void goToPoint(Pose targetPose) {
        if (!follower.isBusy()) {
            PathChain path = follower.pathBuilder()
                    .addPath(new BezierLine(new Point(follower.getPose()), new Point(targetPose)))
                    .setLinearHeadingInterpolation(follower.getPose().getHeading(), targetPose.getHeading())
                    .build();
            follower.followPath(path, true);
        }
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

}