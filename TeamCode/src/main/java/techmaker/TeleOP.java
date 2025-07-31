package techmaker;

import static techmaker.subsystems.IntakeSubsystem.RIGHT_INTAKE_SLIDER_MIN;
import static techmaker.subsystems.IntakeSubsystem.RIGHT_INTAKE_WRIST_MIN;

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

@TeleOp(name = "TeleOp geral")
public class TeleOP extends OpMode {
    private ClawSubsystem claw;
    private ElevatorSubsystem elevator;
    private IntakeSubsystem intake;
    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);
    private StateMachine state = StateMachine.IDLE;
    private StateMachine stateClawSample = StateMachine.CLAW_SPECIMENT;
    private final ElapsedTime timer = new ElapsedTime();
    private long timeout = 0;
    @Override
    public void init() {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        claw = new ClawSubsystem(hardwareMap);
        elevator = new ElevatorSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap, false);
        claw.clawWrist(ClawSubsystem.minWristL, ClawSubsystem.minWristR);
        claw.clawArm(ClawSubsystem.medArml, ClawSubsystem.medArmR);
        intake.intakeWrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MIN, RIGHT_INTAKE_WRIST_MIN);
        intake.intakeWrist(IntakeSubsystem.LEFT_INTAKE_SLIDER_MIN, RIGHT_INTAKE_SLIDER_MIN);


        telemetry.addData("Status", "TeleOp Principal Inicializado");
        telemetry.addData("Dashboard", "Conecte-se em 192.168.43.1:8080");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.setTeleOpMovementVectors(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false
        );

        if(gamepad2.triangle && state == StateMachine.IDLE){
            state = StateMachine.START_INTAKE;
            intake.slider(IntakeSubsystem.LEFT_INTAKE_SLIDER_MAX,IntakeSubsystem.RIGHT_INTAKE_SLIDER_MAX);
            timeout = 40;
            timer.reset();
        }
        if(gamepad2.circle && state == StateMachine.INTAKING){
            state = StateMachine.RETURNING_INTAKE;
            intake.intakeWrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MIN, IntakeSubsystem.RIGHT_INTAKE_WRIST_MIN);
            intake.stopIntake();
            timeout = 40;
            timer.reset();

        }
        if(timer.milliseconds()>timeout){
            if(state==StateMachine.START_INTAKE){
                intake.intakeWrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MAX, IntakeSubsystem.RIGHT_INTAKE_WRIST_MAX);
                intake.startIntake();
                state = StateMachine.INTAKING;
            }
            if(state==StateMachine.RETURNING_INTAKE){
                intake.slider(IntakeSubsystem.LEFT_INTAKE_SLIDER_MIN,IntakeSubsystem.RIGHT_INTAKE_SLIDER_MIN);
                state = StateMachine.IDLE;

            }
            if (gamepad2.right_bumper && stateClawSample == StateMachine.CLAW_SPECIMENT) {
                stateClawSample = StateMachine.CLAW_SAMPLE;

                claw.middleClaw(ClawSubsystem.maxClaw);
                timeout = 40;
                timer.reset();
            }

            if (timer.milliseconds() > timeout) {
                if (stateClawSample == StateMachine.CLAW_SAMPLE) {
                    intake.reverseIntake();
                    stateClawSample = StateMachine.DELIVER_SAMPLE;
                    timeout = 40;
                    timer.reset();

                } else if (stateClawSample == StateMachine.DELIVER_SAMPLE) {
                    claw.clawArm(ClawSubsystem.maxArmL, ClawSubsystem.maxArmR);
                    intake.stopIntake();
                    stateClawSample = StateMachine.DELIVERY_SPECIMENT;
                }
            }

            if (gamepad2.left_bumper && stateClawSample == StateMachine.DELIVERY_SPECIMENT) {
                claw.middleClaw(ClawSubsystem.minClaw);
                stateClawSample = StateMachine.CLAW_RETRACT;
                timeout = 40;
                timer.reset();
            }

            if (timer.milliseconds() > timeout) {
                if (stateClawSample == StateMachine.CLAW_RETRACT) {
                    claw.clawArm(ClawSubsystem.medArml, ClawSubsystem.medArmR);
                    intake.stopIntake();
                    stateClawSample = StateMachine.CLAW_SPECIMENT;
                }
            }
        }

        follower.update();
        claw.update(telemetry);
        elevator.update(telemetry);
        intake.update(telemetry);
        telemetry.update();
    }
}
