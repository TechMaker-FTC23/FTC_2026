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
            intake.slider(IntakeSubsystem.LEFT_INTAKE_SLIDER_MAX, IntakeSubsystem.RIGHT_INTAKE_SLIDER_MAX);
            timeout = 40;
            timer.reset();
        }
        if (gamepad.circle && intakeState == StateMachine.INTAKING) {
            intakeState = StateMachine.RETURNING_INTAKE;
            intake.intakeWrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MIN, IntakeSubsystem.RIGHT_INTAKE_WRIST_MIN);
            intake.stopIntake();
            timeout = 40;
            timer.reset();
        }
        if (timer.milliseconds() > timeout) {
            if (intakeState == StateMachine.START_INTAKE) {
                intake.intakeWrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MAX, IntakeSubsystem.RIGHT_INTAKE_WRIST_MAX);
                intake.startIntake();
                intakeState = StateMachine.INTAKING;
            } else if (intakeState == StateMachine.RETURNING_INTAKE) {
                intake.slider(IntakeSubsystem.LEFT_INTAKE_SLIDER_MIN, IntakeSubsystem.RIGHT_INTAKE_SLIDER_MIN);
                intakeState = StateMachine.IDLE;
            }
        }
    }

    public void updateClaw(Gamepad gamepad) {
        if (gamepad.right_bumper && clawState == StateMachine.CLAW_SPECIMENT) {
            clawState = StateMachine.CLAW_SAMPLE;
            claw.middleClaw(ClawSubsystem.maxClaw);
            timeout = 40;
            timer.reset();
        }

        if (timer.milliseconds() > timeout) {
            if (clawState == StateMachine.CLAW_SAMPLE) {
                intake.reverseIntake();
                claw.clawWrist(ClawSubsystem.maxWristL, ClawSubsystem.maxWristR);
                clawState = StateMachine.DELIVER_SAMPLE;
                timeout = 40;
                timer.reset();
            } else if (clawState == StateMachine.DELIVER_SAMPLE) {
                claw.clawArm(ClawSubsystem.maxArmL, ClawSubsystem.maxArmR);
                intake.stopIntake();
                clawState = StateMachine.DELIVERY_SPECIMENT;
            }
        }

        if (gamepad.left_bumper && clawState == StateMachine.DELIVERY_SPECIMENT) {
            claw.middleClaw(ClawSubsystem.minClaw);
            clawState = StateMachine.CLAW_RETRACT;
            timeout = 40;
            timer.reset();
        }

        if (timer.milliseconds() > timeout && clawState == StateMachine.CLAW_RETRACT) {
            claw.clawArm(ClawSubsystem.medArml, ClawSubsystem.medArmR);
            intake.stopIntake();
            clawState = StateMachine.CLAW_SPECIMENT;
        }
    }

    public void runAutoCycle(Gamepad gamepad) {
        if (gamepad.x && intakeState == StateMachine.IDLE) {
            intakeState = StateMachine.AUTO_CYCLE_START;
            intake.slider(IntakeSubsystem.LEFT_INTAKE_SLIDER_MAX, IntakeSubsystem.RIGHT_INTAKE_SLIDER_MAX);
            timeout = 200;
            timer.reset();
        }

        if (timer.milliseconds() > timeout) {
            if (intakeState == StateMachine.AUTO_CYCLE_START) {
                intake.intakeWrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MAX, IntakeSubsystem.RIGHT_INTAKE_WRIST_MAX);
                intake.startIntake();
                intakeState = StateMachine.AUTO_INTAKING;
                timeout = 400;
                timer.reset();
            } else if (intakeState == StateMachine.AUTO_INTAKING) {
                claw.middleClaw(ClawSubsystem.maxClaw);
                intake.reverseIntake();
                claw.clawWrist(ClawSubsystem.maxWristL, ClawSubsystem.maxWristR);
                intakeState = StateMachine.AUTO_GRAB;
                timeout = 200;
                timer.reset();
            } else if (intakeState == StateMachine.AUTO_GRAB) {
                intake.stopIntake();
                claw.clawArm(ClawSubsystem.maxArmL, ClawSubsystem.maxArmR);
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
