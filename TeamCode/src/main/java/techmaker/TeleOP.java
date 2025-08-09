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
import techmaker.util.StateMachine; // Importa o seu enum atualizado

@TeleOp(name = "TeleOp Final e Corrigido")
public class TeleOP extends OpMode {
    private ClawSubsystem claw;
    private ElevatorSubsystem elevator;
    private IntakeSubsystem intake;
    private Follower follower;

    // A única máquina de estados, usando o seu enum completo.
    private StateMachine state = StateMachine.IDLE;
    private double headingOffset = 0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        claw = new ClawSubsystem(hardwareMap);
        elevator = new ElevatorSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap, false); // Defina a aliança aqui

        // Posição inicial segura do robô
        claw.setState(ClawSubsystem.ClawState.TRAVEL);
        claw.setClawOpen(true);
        elevator.goToPosition(ElevatorSubsystem.ELEVATOR_PRESET_GROUND);
        intake.sliderMin();

        telemetry.addData("Status", "TeleOp Final Inicializado");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        headingOffset = follower.poseUpdater.getPose().getHeading();
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

        double rawHeading = follower.poseUpdater.getPose().getHeading();
        double heading = normalizeAngle(rawHeading - headingOffset);

        double rotatedX = x_stick * Math.cos(heading) + y_stick * Math.sin(heading);
        double rotatedY = -x_stick * Math.sin(heading) + y_stick * Math.cos(heading);

        follower.setTeleOpMovementVectors(rotatedY, rotatedX, turn_stick, false);
    }

    private void handleInputs() {
        // Comandos só são aceites em estados "de espera" para evitar conflitos.
        if (state == StateMachine.IDLE || state == StateMachine.CLAW_SPECIMENT) {
            // Coleta com slider no máximo
            if (gamepad2.triangle) {
                state = StateMachine.START_INTAKE;
            }
            // Coleta com slider no médio
            if (gamepad2.square) {
                state = StateMachine.START_INTAKE_MEDIUM;
            }
            // Inicia o ciclo de pontuação
            if (gamepad2.dpad_up) {
                state = StateMachine.SCORE_HIGH;
            }
            if (gamepad2.dpad_left || gamepad2.dpad_right) {
                state = StateMachine.SCORE_MEDIUM;
            }
            if (gamepad2.dpad_down) {
                state = StateMachine.SCORE_LOW;
            }
            // Inicia o ciclo automático
            if (gamepad2.cross) {
                state = StateMachine.AUTO_CYCLE_START;
            }
        }

        // Comandos de interrupção
        if (gamepad2.right_bumper) {
            claw.setClawOpen(false); // Fecha a garra
        }
        // Solta o pixel e inicia a retração
        if (gamepad2.left_bumper) {
            claw.setClawOpen(true); // Abre a garra para pontuar
            // Se estivermos numa posição de pontuação, podemos iniciar a retração
            if (state == StateMachine.DELIVERY_SPECIMENT) {
                state = StateMachine.CLAW_RETRACT;
            }
        }
        // Comando de "reset" para voltar à posição segura
        if (gamepad2.circle) {
            state = StateMachine.RETURNING_INTAKE;
        }
    }

    private void runStateMachine() {
        switch (state) {
            case IDLE:
            case CLAW_SPECIMENT: // Ambos são estados de espera
                break;

            // --- Ciclos de Intake ---
            case START_INTAKE:
                intake.sliderMax();
                state = StateMachine.INTAKE_SETUP;
                break;
            case START_INTAKE_MEDIUM:
                intake.sliderMedium();
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
                    claw.setClawOpen(false); // Garra fecha no pixel
                    intake.wrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MIN, IntakeSubsystem.RIGHT_INTAKE_WRIST_MIN);
                    state = StateMachine.CLAW_SPECIMENT; // Pixel seguro, pronto para o próximo passo
                }
                break;

            // --- Ciclo de Pontuação ---
            case SCORE_LOW:
            case SCORE_MEDIUM:
            case SCORE_HIGH:
                claw.setState(ClawSubsystem.ClawState.SCORE);
                if(state == StateMachine.SCORE_LOW) elevator.goToPosition(ElevatorSubsystem.ELEVATOR_PRESET_LOW);
                if(state == StateMachine.SCORE_MEDIUM) elevator.goToPosition(ElevatorSubsystem.ELEVATOR_PRESET_MEDIUM);
                if(state == StateMachine.SCORE_HIGH) elevator.goToPosition(ElevatorSubsystem.ELEVATOR_PRESET_HIGH);
                state = StateMachine.WAITING_FOR_LIFT;
                break;
            case WAITING_FOR_LIFT:
                if (!elevator.isBusy()) {
                    state = StateMachine.DELIVERY_SPECIMENT; // Pronto para soltar o pixel
                }
                break;
            case DELIVERY_SPECIMENT:
                // Aguarda o piloto pressionar o bumper esquerdo para soltar
                break;
            case CLAW_RETRACT:
                // O bumper esquerdo já abriu a garra
                state = StateMachine.TRAVELLING;
                break;
            case TRAVELLING:
                claw.setState(ClawSubsystem.ClawState.TRAVEL);
                elevator.goToPosition(ElevatorSubsystem.ELEVATOR_PRESET_GROUND);
                state = StateMachine.INTAKE_RETURNED;
                break;

            // --- Ciclo de Retorno Geral ---
            case RETURNING_INTAKE:
                intake.stopIntake();
                intake.sliderMin();
                claw.setState(ClawSubsystem.ClawState.TRAVEL);
                elevator.goToPosition(ElevatorSubsystem.ELEVATOR_PRESET_GROUND);
                state = StateMachine.INTAKE_RETURNED;
                break;
            case INTAKE_RETURNED:
                if (!elevator.isBusy()) {
                    state = StateMachine.IDLE;
                }
                break;

            // --- Ciclo Automático (Exemplo de como poderia ser) ---
            case AUTO_CYCLE_START:
                intake.sliderMax();
                state = StateMachine.AUTO_INTAKING;
                break;
            case AUTO_INTAKING:
                intake.resetCaptureState();
                intake.wrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MAX, IntakeSubsystem.RIGHT_INTAKE_WRIST_MAX);
                intake.startAutomaticCapture();
                if(intake.isCaptureComplete()){
                    state = StateMachine.AUTO_GRAB;
                }
                break;
            case AUTO_GRAB:
                claw.setClawOpen(false);
                claw.setState(ClawSubsystem.ClawState.INTAKE);
                intake.reverseIntake();
                state = StateMachine.AUTO_RAISE;
                break;
            case AUTO_RAISE:
                claw.setState(ClawSubsystem.ClawState.TRAVEL);
                elevator.goToPosition(ElevatorSubsystem.ELEVATOR_PRESET_LOW);
                state = StateMachine.CLAW_SPECIMENT;
                break;
        }
    }

    private void updateSubsystems() {
        follower.update();
        intake.updateAutomaticCapture();
        intake.maintainSliderPosition();

        intake.update(telemetry);
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
