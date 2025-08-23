package techmaker;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
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

@TeleOp(name = "TeleOp Geral")
public class TeleOP extends OpMode {
    private ClawSubsystem claw;
    private ElevatorSubsystem elevator;
    private IntakeSubsystem intake;
    private Follower follower;
    private Limelight3A limelight;
    private StateMachine state = StateMachine.IDLE;
    private StateMachine stateClawSample = StateMachine.CLAW_SPECIMENT;
    private final Pose startPose = new Pose(0, 0, 180);
    private final Pose BargeUp = new Pose(95 / 2.54, 80 / 2.54, Math.toRadians(0));
    private final Pose BargeMiddle = new Pose(100 / 2.54, 30 / 2.54, Math.toRadians(-90));
    private final Pose Basket = new Pose(51.65671040692668, 58.230110228531004, Math.toRadians(221.45235477987202));
    private final ElapsedTime timer = new ElapsedTime();
    private long timeout = 0;


    @Override
    public void init() {
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(DataStorage.robotPose);
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
       follower.setPose(startPose);
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
        double y = -gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        double heading = follower.getPose().getHeading();

        follower.setTeleOpMovementVectors(y, x, turn, false);
        follower.update();
        updatePoseFromLimelight();

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
        if(gamepad1.dpad_up){
            follower.setPose(startPose);
            follower.update();
            updatePoseFromLimelight();
        }

        if (gamepad2.right_bumper && stateClawSample == StateMachine.CLAW_SPECIMENT) {
            stateClawSample = StateMachine.CLAW_SAMPLE;
            claw.setArmPosition(ClawSubsystem.ARM_LEFT_INTAKE_CLAW, ClawSubsystem.ARM_RIGHT_INTAKE_CLAW);
            claw.setClawOpen(false);
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

        if (state == StateMachine.REVERTING_INTAKE && !intake.isPixelDetected()) {
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
                    timeout = 30;
                    timer.reset();
                } else {
                    intake.reverseIntake();
                    state = StateMachine.REVERTING_INTAKE;
                }
            } else if (state == StateMachine.RETURNING_INTAKE) {
                intake.stopIntake();
                intake.sliderMin();
                state = StateMachine.IDLE;
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
                intake.reverseIntake();
                elevator.goToPositionPID(ElevatorSubsystem.ELEVATOR_PRESET_HIGH);
                stateClawSample = StateMachine.DELIVER_SAMPLE;
                timeout = 200;
                timer.reset();
            } else if (stateClawSample == StateMachine.DELIVER_SAMPLE) {
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
        telemetry.update();

    }

    private void updatePoseFromLimelight() {
        double currentYawDegrees = Math.toDegrees(follower.getPose().getHeading());
        limelight.updateRobotOrientation(currentYawDegrees);

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return;
        }

        Pose3D botpose = result.getBotpose_MT2();
        if (botpose != null) {
            Pose visionPose = new Pose(
                    botpose.getPosition().x * 39.37, // metros para polegadas
                    botpose.getPosition().y * 39.37, // metros para polegadas
                    botpose.getOrientation().getYaw(AngleUnit.RADIANS)
            );
            follower.setPose(visionPose);
            follower.update();
        }
    }
}
