package techmaker;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import techmaker.subsystems.ClawSubsystem;
import techmaker.subsystems.IntakeSubsystem;
import techmaker.util.StateMachine;

public class RobotStateManager {
    private StateMachine intakeState = StateMachine.IDLE;
    private StateMachine clawState = StateMachine.CLAW_SPECIMENT;
    private final ElapsedTime timer = new ElapsedTime();
    private long timeout = 0;
    private final IntakeSubsystem intake;
    private final ClawSubsystem claw;

    public RobotStateManager(IntakeSubsystem intake, ClawSubsystem claw) {
        this.intake = intake;
        this.claw = claw;
    }

    public void updateIntake(Gamepad gamepad) {
        if (gamepad.triangle && intakeState == StateMachine.IDLE) {
            intakeState = StateMachine.START_INTAKE;
            intake.sliderMax();
            timeout = 40;
            timer.reset();
        }
        if (gamepad.circle && intakeState == StateMachine.INTAKING) {
            intakeState = StateMachine.RETURNING_INTAKE;
            intake.wrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MIN, IntakeSubsystem.RIGHT_INTAKE_WRIST_MIN);
            intake.stopIntake();
            timeout = 40;
            timer.reset();
        }
        if (timer.milliseconds() > timeout) {
            if (intakeState == StateMachine.START_INTAKE) {
                intake.wrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MAX, IntakeSubsystem.RIGHT_INTAKE_WRIST_MAX);
                intake.startIntake();
                intakeState = StateMachine.INTAKING;
            } else if (intakeState == StateMachine.RETURNING_INTAKE) {
                intake.sliderMin();
                intakeState = StateMachine.IDLE;
            }
        }
    }

    public void updateClaw(Gamepad gamepad) {
        if (gamepad.right_bumper && clawState == StateMachine.CLAW_SPECIMENT) {
            clawState = StateMachine.CLAW_SAMPLE;
            // MUDANÇA: Usa o novo método para fechar a garra.
            claw.setClawOpen(false);
            timeout = 40;
            timer.reset();
        }

        if (timer.milliseconds() > timeout) {
            if (clawState == StateMachine.CLAW_SAMPLE) {
                intake.reverseIntake();
                // MUDANÇA: Define a posição de pontuação com um único comando.
                claw.setState(ClawSubsystem.ClawState.SCORE);
                clawState = StateMachine.DELIVER_SAMPLE;
                timeout = 40;
                timer.reset();
            } else if (clawState == StateMachine.DELIVER_SAMPLE) {
                // A ação de mover os servos já foi feita no estado anterior.
                // Este estado agora apenas espera.
                intake.stopIntake();
                clawState = StateMachine.DELIVERY_SPECIMENT;
            }
        }

        if (gamepad.left_bumper && clawState == StateMachine.DELIVERY_SPECIMENT) {
            // MUDANÇA: Usa o novo método para abrir a garra.
            claw.setClawOpen(true);
            clawState = StateMachine.CLAW_RETRACT;
            timeout = 40;
            timer.reset();
        }

        if (timer.milliseconds() > timeout && clawState == StateMachine.CLAW_RETRACT) {
            // MUDANÇA: Define a posição de viagem com um único comando.
            claw.setState(ClawSubsystem.ClawState.TRAVEL);
            intake.stopIntake();
            clawState = StateMachine.CLAW_SPECIMENT;
        }
    }

    public void runAutoCycle(Gamepad gamepad) {
        if (gamepad.x && intakeState == StateMachine.IDLE) {
            intakeState = StateMachine.AUTO_CYCLE_START;
            intake.sliderMax();
            timeout = 200;
            timer.reset();
        }

        if (timer.milliseconds() > timeout) {
            if (intakeState == StateMachine.AUTO_CYCLE_START) {
                intake.wrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MAX, IntakeSubsystem.RIGHT_INTAKE_WRIST_MAX);
                intake.startIntake();
                intakeState = StateMachine.AUTO_INTAKING;
                timeout = 400;
                timer.reset();
            } else if (intakeState == StateMachine.AUTO_INTAKING) {
                // MUDANÇA: Usa a nova API da garra.
                claw.setClawOpen(false);
                claw.setState(ClawSubsystem.ClawState.INTAKE);
                intake.reverseIntake();
                intakeState = StateMachine.AUTO_GRAB;
                timeout = 200;
                timer.reset();
            } else if (intakeState == StateMachine.AUTO_GRAB) {
                intake.stopIntake();
                // MUDANÇA: Usa a nova API da garra.
                claw.setState(ClawSubsystem.ClawState.SCORE);
                intakeState = StateMachine.AUTO_RAISE;
                timeout = 200;
                timer.reset();
            } else if (intakeState == StateMachine.AUTO_RAISE) {
                intakeState = StateMachine.IDLE;
            }
        }
    }

    public StateMachine getIntakeState() {
        return intakeState;
    }

    public StateMachine getClawState() {
        return clawState;
    }
}
