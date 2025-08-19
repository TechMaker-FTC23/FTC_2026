package techmaker;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import techmaker.constants.FConstants;
import techmaker.constants.LConstants;
import techmaker.subsystems.ClawSubsystem;
import techmaker.subsystems.ElevatorSubsystem;
import techmaker.subsystems.IntakeSubsystem;
import techmaker.util.StateMachine;

@TeleOp(name = "TeleOp Adaptado para Novo Elevador")
public class TeleOP extends OpMode {
    private ClawSubsystem claw;
    private ElevatorSubsystem elevator;
    private IntakeSubsystem intake;
    private Follower follower;

    private StateMachine state = StateMachine.IDLE;
    private double headingOffset = 0;

    // NOVO: Variável para guardar a posição alvo do elevador,
    // já que a nova classe não tem um método getTargetPosition().
    private int elevatorTargetPosition;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        claw = new ClawSubsystem(hardwareMap);
        elevator = new ElevatorSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap, false);

        claw.setState(ClawSubsystem.ClawState.TRAVEL);
        claw.setClawOpen(true);

        // MUDANÇA: Usa o novo método do elevador e guarda a posição alvo.
        elevatorTargetPosition = ElevatorSubsystem.ELEVATOR_PRESET_GROUND;
        elevator.goToPositionPID(elevatorTargetPosition);

        intake.sliderMin();

        telemetry.addData("Status", "TeleOp Adaptado Inicializado");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        headingOffset = follower.getPose().getHeading();
        state = StateMachine.IDLE;
    }

    @Override
    public void loop() {
        handleDrive();
        handleInputs();
        runStateMachine();
        updateSubsystems();
    }

    private void handleDrive() {
        double y_stick = -gamepad1.left_stick_y;
        double x_stick = gamepad1.left_stick_x;
        double turn_stick = -gamepad1.right_stick_x;

        double rawHeading = follower.getPose().getHeading();
        double heading = normalizeAngle(rawHeading - headingOffset);

        double rotatedX = x_stick * Math.cos(heading) + y_stick * Math.sin(heading);
        double rotatedY = -x_stick * Math.sin(heading) + y_stick * Math.cos(heading);

        follower.setTeleOpMovementVectors(rotatedY, rotatedX, turn_stick, true);
    }

    private void handleInputs() {
        if (state == StateMachine.IDLE || state == StateMachine.CLAW_SPECIMENT) {
            if (gamepad2.triangle) {
                state = StateMachine.START_INTAKE;
            }
            if (gamepad2.dpad_up) state = StateMachine.SCORE_HIGH;
            if (gamepad2.dpad_left || gamepad2.dpad_right) state = StateMachine.SCORE_MEDIUM;
            if (gamepad2.dpad_down) state = StateMachine.SCORE_LOW;
        }

        if (gamepad2.right_bumper) claw.setClawOpen(false);
        if (gamepad2.left_bumper) {
            claw.setClawOpen(true);
            if (state == StateMachine.DELIVERY_SPECIMENT) {
                state = StateMachine.CLAW_RETRACT;
            }
        }
        if (gamepad2.circle) {
            state = StateMachine.RETURNING_INTAKE;
        }
    }

    private void runStateMachine() {
        switch (state) {
            case IDLE:
            case CLAW_SPECIMENT:
                break;

            case START_INTAKE:
                intake.sliderMax();
                state = StateMachine.INTAKE_SETUP;
                break;
            case INTAKE_SETUP:
                intake.resetCaptureState();
                intake.wrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MAX, IntakeSubsystem.RIGHT_INTAKE_WRIST_MAX);
                intake.startAutomaticCapture();
                state = StateMachine.INTAKING;
                break;
            case INTAKING:
                if (intake.isCaptureComplete()) {
                    gamepad1.rumble(200);
                    claw.setClawOpen(false);
                    intake.wrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MIN, IntakeSubsystem.RIGHT_INTAKE_WRIST_MIN);
                    state = StateMachine.CLAW_SPECIMENT;
                }
                break;

            case SCORE_LOW:
            case SCORE_MEDIUM:
            case SCORE_HIGH:
                claw.setState(ClawSubsystem.ClawState.SCORE);
                // MUDANÇA: Usa o novo método do elevador e guarda a posição alvo.
                if (state == StateMachine.SCORE_LOW) elevatorTargetPosition = ElevatorSubsystem.ELEVATOR_PRESET_LOW;
                if (state == StateMachine.SCORE_MEDIUM) elevatorTargetPosition = ElevatorSubsystem.ELEVATOR_PRESET_MEDIUM;
                if (state == StateMachine.SCORE_HIGH) elevatorTargetPosition = ElevatorSubsystem.ELEVATOR_PRESET_HIGH;
                elevator.goToPositionPID(elevatorTargetPosition);
                state = StateMachine.WAITING_FOR_LIFT;
                break;
            case WAITING_FOR_LIFT:
                // MUDANÇA: Compara a posição atual com a posição alvo guardada.
                if (Math.abs(elevator.getCurrentPosition() - elevatorTargetPosition) < 20) {
                    state = StateMachine.DELIVERY_SPECIMENT;
                }
                break;
            case DELIVERY_SPECIMENT:
                break;
            case CLAW_RETRACT:
                state = StateMachine.TRAVELLING;
                break;
            case TRAVELLING:
                claw.setState(ClawSubsystem.ClawState.TRAVEL);
                // MUDANÇA: Usa o novo método do elevador e guarda a posição alvo.
                elevatorTargetPosition = ElevatorSubsystem.ELEVATOR_PRESET_GROUND;
                elevator.goToPositionPID(elevatorTargetPosition);
                state = StateMachine.INTAKE_RETURNED;
                break;

            case RETURNING_INTAKE:
                intake.stopIntake();
                intake.sliderMin();
                claw.setState(ClawSubsystem.ClawState.TRAVEL);
                // MUDANÇA: Usa o novo método do elevador e guarda a posição alvo.
                elevatorTargetPosition = ElevatorSubsystem.ELEVATOR_PRESET_GROUND;
                elevator.goToPositionPID(elevatorTargetPosition);
                state = StateMachine.INTAKE_RETURNED;
                break;
            case INTAKE_RETURNED:
                // MUDANÇA: Compara a posição atual com a posição alvo guardada.
                if (Math.abs(elevator.getCurrentPosition() - elevatorTargetPosition) < 20) {
                    state = StateMachine.IDLE;
                }
                break;
        }
    }

    private void updateSubsystems() {
        follower.update();
        intake.updateAutomaticCapture();
        intake.maintainSliderPosition();

        intake.update(telemetry);
        // A chamada ao update do elevador agora executa o PID.
        elevator.update(telemetry);
        claw.update(telemetry);

        telemetry.addData("Estado Atual", state);
        telemetry.update();
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
