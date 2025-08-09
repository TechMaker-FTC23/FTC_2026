package techmaker;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import techmaker.constants.FConstants;
import techmaker.constants.LConstants;
import techmaker.subsystems.ClawSubsystem;
import techmaker.subsystems.ElevatorSubsystem;
import techmaker.subsystems.IntakeSubsystem;
import techmaker.util.StateMachine;

@TeleOp(name = "TeleOp Geral")
public class TeleOp2 extends OpMode {
    private ClawSubsystem claw;
    private ElevatorSubsystem elevator;
    private IntakeSubsystem intake;
    private Follower follower;

    private StateMachine state = StateMachine.IDLE;
    private StateMachine stateClawSample = StateMachine.CLAW_SPECIMENT;

    private final ElapsedTime timer = new ElapsedTime();
    private long timeout = 0;

    private double headingOffset = 0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(new Pose(0, 0, 0));

        claw = new ClawSubsystem(hardwareMap);
        elevator = new ElevatorSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap, false);

        // MUDANÇA: Define a posição inicial segura da garra com um único comando.
        claw.setState(ClawSubsystem.ClawState.TRAVEL);
        claw.setClawOpen(true); // Começa com a garra aberta
        claw.setArmPosition(ClawSubsystem.ARM_LEFT_TRAVEL_CLAW, ClawSubsystem.ARM_RIGHT_TRAVEL_CLAW);

        intake.wrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MIN, IntakeSubsystem.RIGHT_INTAKE_WRIST_MIN);
        intake.sliderMin();

        telemetry.addData("Status", "TeleOp Principal Inicializado");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        timer.reset();
        headingOffset = follower.poseUpdater.getPose().getHeading();
    }

    @Override
    public void loop() {
        double y_stick = gamepad1.left_stick_y;
        double x_stick = -gamepad1.left_stick_x;
        double turn_stick = -gamepad1.right_stick_x;

        follower.setTeleOpMovementVectors(y_stick, x_stick, turn_stick, false);

        intake.maintainSliderPosition();

        if (gamepad2.triangle && state == StateMachine.IDLE) {
            state = StateMachine.START_INTAKE;
            intake.wrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MAX, IntakeSubsystem.RIGHT_INTAKE_WRIST_MAX);
            intake.startIntake();
            timeout = 200;
            timer.reset();
        } else if ((gamepad2.circle && state == StateMachine.INTAKING) ||
                intake.isPixelDetected() && state==StateMachine.INTAKING){
            state = StateMachine.RETURNING_INTAKE;
            intake.wrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MIN, IntakeSubsystem.RIGHT_INTAKE_WRIST_MIN);
            intake.stopIntake();
            timeout = 100;
            timer.reset();
        }

        if (gamepad2.right_bumper && stateClawSample == StateMachine.CLAW_SPECIMENT) {
            stateClawSample = StateMachine.CLAW_SAMPLE;
            claw.setArmPosition(ClawSubsystem.ARM_LEFT_INTAKE_CLAW, ClawSubsystem.ARM_RIGHT_INTAKE_CLAW);
            claw.setClawOpen(false);
            timeout = 100;
            timer.reset();
        } else if (gamepad2.left_bumper && stateClawSample == StateMachine.DELIVERY_SPECIMENT) {
            stateClawSample = StateMachine.CLAW_RETRACT;
            // MUDANÇA: Usa o novo método para abrir a garra.
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

        if (gamepad2.dpad_right){
            intake.reverseIntake();
        }

        if (timer.milliseconds() > timeout) {
            if (state == StateMachine.START_INTAKE) {
                intake.sliderMax();

                state = StateMachine.INTAKING;
            } else if (state == StateMachine.RETURNING_INTAKE) {
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
            }
            else if (stateClawSample == StateMachine.DELIVER_SAMPLE) {
                claw.setState(ClawSubsystem.ClawState.SCORE);
                intake.stopIntake();
                stateClawSample = StateMachine.DELIVERY_SPECIMENT;
            }
            else if (stateClawSample == StateMachine.CLAW_RETRACT) {
                claw.setState(ClawSubsystem.ClawState.TRAVEL);
                elevator.goToPositionPID(ElevatorSubsystem.ELEVATOR_PRESET_GROUND);
                intake.stopIntake();
                stateClawSample = StateMachine.CLAW_SPECIMENT;
            }
        }

        follower.update();
        claw.update(telemetry);
        elevator.update(telemetry);
        intake.update(telemetry);
        telemetry.addData("Main State", state);
        telemetry.addData("Claw State", stateClawSample);
        telemetry.update();
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
