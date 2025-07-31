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

@TeleOp(name = "TeleOp Geral (IF/ELSE IF)")
public class TeleOp2 extends OpMode {
    private ClawSubsystem claw;
    private ElevatorSubsystem elevator;
    private IntakeSubsystem intake;
    private Follower follower;

    private StateMachine state = StateMachine.IDLE;
    private StateMachine stateClawSample = StateMachine.CLAW_SPECIMENT;

    private final ElapsedTime timer = new ElapsedTime();
    private long timeout = 0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(new Pose(0, 0, 0));

        claw = new ClawSubsystem(hardwareMap);
        elevator = new ElevatorSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap, false);

        claw.clawWrist(ClawSubsystem.minWristL, ClawSubsystem.minWristR);
        claw.clawArm(ClawSubsystem.medArml, ClawSubsystem.medArmR);
        intake.intakeWrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MIN, IntakeSubsystem.RIGHT_INTAKE_WRIST_MIN);
        intake.slider(IntakeSubsystem.LEFT_INTAKE_SLIDER_MIN, IntakeSubsystem.RIGHT_INTAKE_SLIDER_MIN);

        telemetry.addData("Status", "TeleOp Principal Inicializado");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        timer.reset();
    }

    @Override
    public void loop() {
        double y_stick = -gamepad1.left_stick_y;
        double x_stick = gamepad1.left_stick_x;
        double turn_stick = -gamepad1.right_stick_x;
        double heading = follower.poseUpdater.getPose().getHeading();
        double rotatedX = x_stick * Math.cos(-heading) - y_stick * Math.sin(-heading);
        double rotatedY = x_stick * Math.sin(-heading) + y_stick * Math.cos(-heading);
        follower.setTeleOpMovementVectors(rotatedY, rotatedX, turn_stick, false);

        if (gamepad2.triangle && state == StateMachine.IDLE) {
            state = StateMachine.START_INTAKE;
            intake.slider(IntakeSubsystem.LEFT_INTAKE_SLIDER_MAX, IntakeSubsystem.RIGHT_INTAKE_SLIDER_MAX);
            timeout = 200;
            timer.reset();
        } else if (gamepad2.circle && state == StateMachine.INTAKING) {
            state = StateMachine.RETURNING_INTAKE;
            intake.intakeWrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MIN, IntakeSubsystem.RIGHT_INTAKE_WRIST_MIN);
            intake.stopIntake();
            timeout = 200;
            timer.reset();
        }

        if (gamepad2.right_bumper && stateClawSample == StateMachine.CLAW_SPECIMENT) {
            stateClawSample = StateMachine.CLAW_SAMPLE;
            claw.middleClaw(ClawSubsystem.maxClaw);
            timeout = 200;
            timer.reset();
        } else if (gamepad2.left_bumper && stateClawSample == StateMachine.DELIVERY_SPECIMENT) {
            stateClawSample = StateMachine.CLAW_RETRACT;
            claw.middleClaw(ClawSubsystem.minClaw);
            timeout = 200;
            timer.reset();
        }

        if (gamepad2.x && state == StateMachine.IDLE) {
            state = StateMachine.AUTO_CYCLE_START;
            intake.slider(IntakeSubsystem.LEFT_INTAKE_SLIDER_MAX, IntakeSubsystem.RIGHT_INTAKE_SLIDER_MAX);
            timeout = 200;
            timer.reset();
        }

        if (timer.milliseconds() > timeout) {
            if (state == StateMachine.START_INTAKE) {
                intake.intakeWrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MAX, IntakeSubsystem.RIGHT_INTAKE_WRIST_MAX);
                intake.startIntake();
                state = StateMachine.INTAKING;
            } else if (state == StateMachine.RETURNING_INTAKE) {
                intake.slider(IntakeSubsystem.LEFT_INTAKE_SLIDER_MIN, IntakeSubsystem.RIGHT_INTAKE_SLIDER_MIN);
                state = StateMachine.IDLE;
            } else if (state == StateMachine.AUTO_CYCLE_START) {
                intake.intakeWrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MAX, IntakeSubsystem.RIGHT_INTAKE_WRIST_MAX);
                intake.startIntake();
                state = StateMachine.AUTO_INTAKING;
                timeout = 400;
                timer.reset();
            } else if (state == StateMachine.AUTO_INTAKING) {
                claw.middleClaw(ClawSubsystem.maxClaw);
                intake.reverseIntake();
                claw.clawWrist(ClawSubsystem.maxWristL, ClawSubsystem.maxWristR);
                state = StateMachine.AUTO_GRAB;
                timeout = 200;
                timer.reset();
            } else if (state == StateMachine.AUTO_GRAB) {
                intake.stopIntake();
                claw.clawArm(ClawSubsystem.maxArmL, ClawSubsystem.maxArmR);
                state = StateMachine.AUTO_RAISE;
                timeout = 200;
                timer.reset();
            } else if (state == StateMachine.AUTO_RAISE) {
                state = StateMachine.IDLE;
            }

            if (stateClawSample == StateMachine.CLAW_SAMPLE) {
                intake.reverseIntake();
                stateClawSample = StateMachine.DELIVER_SAMPLE;
                timeout = 200;
                timer.reset();
            } else if (stateClawSample == StateMachine.DELIVER_SAMPLE) {
                claw.clawArm(ClawSubsystem.maxArmL, ClawSubsystem.maxArmR);
                intake.stopIntake();
                stateClawSample = StateMachine.DELIVERY_SPECIMENT;
            } else if (stateClawSample == StateMachine.CLAW_RETRACT) {
                claw.clawArm(ClawSubsystem.medArml, ClawSubsystem.medArmR);
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
}