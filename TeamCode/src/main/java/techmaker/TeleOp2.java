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
import techmaker.util.StateMachine; // Importando o seu enum

@TeleOp(name = "TeleOp Geral Adaptado")
public class TeleOp2 extends OpMode {
    private ClawSubsystem claw;
    private ElevatorSubsystem elevator;
    private IntakeSubsystem intake;
    private Follower follower;
    private StateMachine state = StateMachine.IDLE;

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

        // Posições iniciais do robô
        claw.clawWrist(ClawSubsystem.medWristl, ClawSubsystem.medWristR);
        claw.clawArm(ClawSubsystem.medArml, ClawSubsystem.medArmR);
        claw.middleClaw(ClawSubsystem.minClaw);
        intake.intakeWrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MIN, IntakeSubsystem.RIGHT_INTAKE_WRIST_MIN);
        intake.sliderMin();

        telemetry.addData("Status", "TeleOp Adaptado Inicializado");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        timer.reset();
        headingOffset = follower.poseUpdater.getPose().getHeading();
        state = StateMachine.IDLE;
    }

    @Override
    public void loop() {
        double y_stick = -gamepad1.left_stick_y;
        double x_stick = gamepad1.left_stick_x;
        double turn_stick = -gamepad1.right_stick_x;

        double rawHeading = follower.poseUpdater.getPose().getHeading();
        double heading = normalizeAngle(rawHeading - headingOffset);

        double rotatedX = x_stick * Math.cos(heading) - y_stick * Math.sin(heading);
        double rotatedY = x_stick * Math.sin(heading) + y_stick * Math.cos(heading);

        follower.setTeleOpMovementVectors(-rotatedY, rotatedX, turn_stick, false);

        handleInputs();

        runStateMachine();

        follower.update();
        claw.update(telemetry);
        elevator.update(telemetry);
        intake.update(telemetry);
        telemetry.addData("Main State", state);
        telemetry.update();
    }

    private void handleInputs() {
        if (gamepad2.triangle && state == StateMachine.IDLE) {
            state = StateMachine.START_INTAKE;
        }
        else if (gamepad2.circle && state == StateMachine.INTAKING) {
            state = StateMachine.RETURNING_INTAKE;
        }

        if (gamepad2.square && state == StateMachine.IDLE) {
            state = StateMachine.START_INTAKE_MEDIUM;
        }

        if (gamepad2.cross && state == StateMachine.IDLE) {
            state = StateMachine.AUTO_CYCLE_START;
        }

        if (gamepad2.right_bumper && state == StateMachine.CLAW_SPECIMENT) {
            state = StateMachine.CLAW_SAMPLE;
        }

        if (gamepad2.left_bumper && state == StateMachine.DELIVERY_SPECIMENT) {
            state = StateMachine.CLAW_RETRACT;
        }
    }

    private void runStateMachine() {
        intake.maintainSliderPosition();

        switch (state) {
            case IDLE:
                break;

            case START_INTAKE:
                intake.sliderMax();
                timer.reset();
                timeout = 100;
                state = StateMachine.INTAKE_AVANCE;
                break;
            case INTAKE_AVANCE:
                if (timer.milliseconds() > timeout) {
                    intake.intakeWrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MAX, IntakeSubsystem.RIGHT_INTAKE_WRIST_MAX);
                    state = StateMachine.INTAKING;
                }
                break;
            case INTAKING:
                intake.startIntake();
                break;
            case RETURNING_INTAKE:
                intake.intakeWrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MIN, IntakeSubsystem.RIGHT_INTAKE_WRIST_MIN);
                intake.stopIntake();
                timer.reset();
                timeout = 100;
                state = StateMachine.INTAKE_RETURNED;
                break;
            case INTAKE_RETURNED:
                if (timer.milliseconds() > timeout) {
                    intake.sliderMin();
                    state = StateMachine.IDLE;
                }
                break;

            case START_INTAKE_MEDIUM:
                intake.sliderMedium();
                timeout = 100;
                state = StateMachine.INTAKE_MEDIUM_RETURNED;
                break;
            case INTAKE_MEDIUM_RETURNED:
                if (timer.milliseconds() > timeout) {
                    state = StateMachine.IDLE;
                }
                break;

            case AUTO_CYCLE_START:
                intake.sliderMax();
                timer.reset();
                timeout = 100;
                state = StateMachine.AUTO_INTAKING;
                break;
            case AUTO_INTAKING:
                if (timer.milliseconds() > timeout) {
                    intake.intakeWrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MAX, IntakeSubsystem.RIGHT_INTAKE_WRIST_MAX);
                    intake.startIntake();
                    timer.reset();
                    timeout = 400;
                    state = StateMachine.AUTO_GRAB;
                }
                break;
            case AUTO_GRAB:
                if (timer.milliseconds() > timeout) {
                    claw.middleClaw(ClawSubsystem.maxClaw);
                    intake.reverseIntake();
                    claw.clawWrist(ClawSubsystem.maxWristL, ClawSubsystem.maxWristR);
                    timer.reset();
                    timeout = 200;
                    state = StateMachine.AUTO_RAISE;
                }
                break;
            case AUTO_RAISE:
                if (timer.milliseconds() > timeout) {
                    intake.stopIntake();
                    claw.clawArm(ClawSubsystem.maxArmL, ClawSubsystem.maxArmR);
                    state = StateMachine.CLAW_SPECIMENT;
                }
                break;

            case CLAW_SPECIMENT:
                break;
            case CLAW_SAMPLE:
                claw.middleClaw(ClawSubsystem.maxClaw);
                elevator.goToPositionPID(ElevatorSubsystem.ELEVATOR_PRESET_MEDIUM);
                timer.reset();
                timeout = 200;
                state = StateMachine.DELIVER_SAMPLE;
                break;
            case DELIVER_SAMPLE:
                if (timer.milliseconds() > timeout) {
                    claw.clawArm(ClawSubsystem.maxArmL, ClawSubsystem.maxArmR);
                    state = StateMachine.DELIVERY_SPECIMENT;
                }
                break;
            case DELIVERY_SPECIMENT:
                break;
            case CLAW_RETRACT:
                claw.middleClaw(ClawSubsystem.minClaw);
                timer.reset();
                timeout = 200;
                state = StateMachine.TRAVELLING;
                break;
            case TRAVELLING:
                if (timer.milliseconds() > timeout) {
                    claw.clawArm(ClawSubsystem.medArml, ClawSubsystem.medArmR);
                    elevator.goToPositionPID(ElevatorSubsystem.ELEVATOR_PRESET_GROUND);
                    state = StateMachine.CLAW_SPECIMENT;
                }
                break;
        }
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
