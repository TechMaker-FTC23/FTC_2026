package techmaker;

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
            timeout = 120;
            timer.reset();
        }
        if(gamepad2.circle && state == StateMachine.INTAKING){
            state = StateMachine.RETURNING_INTAKE;
            intake.intakeWrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MIN, IntakeSubsystem.RIGHT_INTAKE_WRIST_MIN);
            intake.stopIntake();
            timeout = 120;
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
        }



        follower.update();
        claw.update(telemetry);
        elevator.update(telemetry);
        intake.update(telemetry);
        telemetry.update();
    }
}
