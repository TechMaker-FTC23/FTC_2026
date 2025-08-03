package techmaker;

import static techmaker.subsystems.IntakeSubsystem.RIGHT_INTAKE_WRIST_MIN;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.follower.Follower;

import techmaker.constants.FConstants;
import techmaker.constants.LConstants;

import techmaker.subsystems.ClawSubsystem;
import techmaker.subsystems.ElevatorSubsystem;
import techmaker.subsystems.IntakeSubsystem;

@TeleOp(name = "Controle Principal do Robô", group = "PedroPathing")
public class RobotMechanisms extends OpMode {

    private ClawSubsystem claw;
    private ElevatorSubsystem elevator;
    private IntakeSubsystem intake;
    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);

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
        intake.sliderMin();

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
        follower.update();

        // Controles do intake
        if (gamepad2.dpad_right) {
            intake.intakeWrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MAX, IntakeSubsystem.RIGHT_INTAKE_WRIST_MAX);
        }
        if (gamepad2.dpad_left) {
            intake.intakeWrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MIN, RIGHT_INTAKE_WRIST_MIN);
            intake.stopIntake();
        }
        if (gamepad2.left_stick_x > 0.5) {
            intake.sliderMax();
        } else if (gamepad2.left_stick_x < -0.5) {
            intake.sliderMin();
        }
        if (gamepad2.triangle){
            intake.intakeWrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MAX,IntakeSubsystem.RIGHT_INTAKE_WRIST_MAX);
            intake.startIntake();
        } else if (gamepad2.right_bumper) {
            claw.middleClaw(ClawSubsystem.maxClaw);
        } else {
            intake.stopIntake();
        }

        // Controles da garra
        if (gamepad2.dpad_down) {
            claw.clawArm(ClawSubsystem.medArml, ClawSubsystem.medArmR);
            claw.clawWrist(ClawSubsystem.medWristl, ClawSubsystem.medWristR);

        }
        if (gamepad2.dpad_up) {
            claw.clawArm(ClawSubsystem.maxArmL, ClawSubsystem.maxArmR);
            claw.clawWrist(ClawSubsystem.maxWristL, ClawSubsystem.maxWristR);
            intake.reverseIntake();

        }

        if (gamepad2.left_bumper) {
            claw.middleClaw(ClawSubsystem.minClaw);
        }

        // Controles do elevador com presets
        if (gamepad2.right_trigger > 0.5) {
            elevator.goToPositionPID(ElevatorSubsystem.ELEVATOR_PRESET_MEDIUM);
        } else if (gamepad2.left_trigger > 0.5) {
            elevator.goToPositionPID(ElevatorSubsystem.ELEVATOR_PRESET_GROUND);
        }

        claw.update(telemetry);
        elevator.update(telemetry);
        intake.update(telemetry);
        telemetry.update();
    }
}
