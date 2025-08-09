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

@TeleOp
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

        // --- MUDANÇA: Configuração inicial da garra usando a nova API ---
        // Define a posição inicial segura com um único comando.
        claw.setState(ClawSubsystem.ClawState.TRAVEL);
        claw.setClawOpen(true); // Começa com a garra aberta

        intake.wrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MIN, RIGHT_INTAKE_WRIST_MIN);

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

        // --- Lógica do Intake (permanece inalterada) ---
        if (gamepad2.dpad_right) {
            intake.wrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MAX, IntakeSubsystem.RIGHT_INTAKE_WRIST_MAX);
        }
        if (gamepad2.dpad_left) {
            intake.wrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MIN, RIGHT_INTAKE_WRIST_MIN);
            intake.stopIntake();
        }
        if (gamepad2.left_stick_x > 0.5) {
            intake.sliderMax();
        } else {
            intake.sliderMin();
        }
        if (gamepad2.triangle){
            intake.wrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MAX,IntakeSubsystem.RIGHT_INTAKE_WRIST_MAX);
            intake.startIntake();
        } else {
            // Pequena correção: o right_bumper não deve parar o intake.
            // A parada do intake deve ser explícita.
            if (!gamepad2.triangle) {
                intake.stopIntake();
            }
        }

        // --- MUDANÇA: Controles da garra usando a nova API ---
        // Move a garra para a posição de VIAGEM (segura)
        if (gamepad2.dpad_down) {
            claw.setState(ClawSubsystem.ClawState.TRAVEL);
        }
        // Move a garra para a posição de PONTUAR
        if (gamepad2.dpad_up) {
            claw.setState(ClawSubsystem.ClawState.SCORE);
            intake.reverseIntake(); // A lógica de reverter o intake permanece
        }

        // Fecha a garra
        if (gamepad2.right_bumper) {
            claw.setClawOpen(false);
        }
        // Abre a garra
        if (gamepad2.left_bumper) {
            claw.setClawOpen(true);
        }

        // --- Lógica do Elevador (permanece inalterada) ---
        if (gamepad2.right_trigger > 0.5) {
            elevator.goToPositionPID(ElevatorSubsystem.ELEVATOR_PRESET_HIGH);
        } else if (gamepad2.left_trigger > 0.5) {
            elevator.goToPositionPID(ElevatorSubsystem.ELEVATOR_PRESET_GROUND);
        }

        claw.update(telemetry);
        elevator.update(telemetry);
        intake.update(telemetry);
        telemetry.update();
    }
}