package pedroPathing.examples;

// NOVO: Importações para integrar o FTC Dashboard
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.pedropathing.follower.Follower;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

import pedroPathing.subsystems.ClawSubsystem;
import pedroPathing.subsystems.ElevatorSubsystem;
import pedroPathing.subsystems.WristSubsystem;

@TeleOp(name = "Controle Principal do Robô", group = "PedroPathing")
public class RobotMechanisms extends OpMode {

    private Follower follower;

    // --- Declaração dos Subsistemas ---
    // ALTERADO: A variável agora segue a convenção de nomes Java (letra minúscula)
    private ClawSubsystem claw;
    private ElevatorSubsystem elevator;
    private WristSubsystem wrist;

    @Override
    public void init() {
        // NOVO: Inicializa o Dashboard e combina a telemetria
        // Agora, tudo que você enviar via telemetry.addData irá para o Driver Hub E para o Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);

        // --- Inicialização dos Subsistemas ---
        claw = new ClawSubsystem(hardwareMap); // ALTERADO: Usa a nova variável
        elevator = new ElevatorSubsystem(hardwareMap);
        wrist = new WristSubsystem(hardwareMap);

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
        // --- CONTROLE DO INTAKE (Gamepad 1) ---
        if (gamepad1.right_trigger > 0.1) {
            claw.runIntake(); // ALTERADO
        } else if (gamepad1.left_trigger > 0.1) {
            claw.runOuttake(); // ALTERADO
        } else {
            claw.stopIntake(); // ALTERADO
        }

        // --- CONTROLE DO WRIST (Gamepad 1) ---
        if (gamepad1.y) { // Mudei para 'Y' para um layout mais intuitivo (cima/abrir)
            wrist.openWrist();
        }
        if (gamepad1.a) { // 'A' para baixo/fechar
            wrist.closeWrist();
        }
        if (gamepad1.b) {
            wrist.setWristMedium();
        }

        // --- CONTROLE DA GARRA DIFERENCIAL (Gamepad 2) ---
        if (gamepad2.x) {
            claw.openClaw(); // ALTERADO
        } else if (gamepad2.b) {
            claw.closeClaw(); // ALTERADO
        } else {
            double grip = -gamepad2.left_stick_y;
            double rotation = gamepad2.right_stick_x;
            claw.controlClaw(grip, rotation); // ALTERADO
        }

        // --- CONTROLE DO ELEVADOR (Gamepad 2) ---
        if (Math.abs(gamepad2.right_stick_y) > 0.1 && !elevator.isMovingToPreset()) {
            elevator.setManualPower(-gamepad2.right_stick_y); // Removida multiplicação extra
        } else if (!elevator.isMovingToPreset()) {
            elevator.setManualPower(0);
        }
        if (gamepad2.a) elevator.goToPositionPID(ElevatorSubsystem.ELEVATOR_PRESET_LOW);
        if (gamepad2.y) elevator.goToPositionPID(ElevatorSubsystem.ELEVATOR_PRESET_MEDIUM);
        if (gamepad2.right_bumper) elevator.goToPositionPID(ElevatorSubsystem.ELEVATOR_PRESET_HIGH);
        if (gamepad2.dpad_down) elevator.goToPositionPID(ElevatorSubsystem.ELEVATOR_PRESET_GROUND);


        // --- ATUALIZAÇÃO DOS SUBSISTEMAS ---
        elevator.update();
        wrist.updateWrist();

        // --- TELEMETRIA ---
        // Estes dados agora aparecem no Driver Hub e no Dashboard
        telemetry.addData("Elevador Posição Alvo", elevator.getTargetPosition());
        telemetry.addData("Elevador Posição Atual", elevator.getCurrentPosition());
        telemetry.addData("Wrist Posição", wrist.getWrist1Position());

        // NOVO: A telemetria acima será automaticamente plotada em um gráfico no Dashboard,
        // permitindo que você visualize o comportamento do PID em tempo real.
        telemetry.update();
    }
}