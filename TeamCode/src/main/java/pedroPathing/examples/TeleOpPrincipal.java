package pedroPathing.examples; // Seu pacote principal

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.pedropathing.follower.Follower; // Para o drivetrain
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

// Importar os subsistemas
import pedroPathing.subsystems.ArmSubsystem;
import pedroPathing.subsystems.ClawSubsystem;
import pedroPathing.subsystems.ElevatorSubsystem;

@TeleOp(name = "TeleOp Principal Com Subsistemas", group = "Principal")
public class TeleOpPrincipal extends OpMode {

    // Drivetrain
    private Follower follower;

    // Subsistemas
    private pedroPathing.subsystems.ClawSubsystem claw;
    private pedroPathing.subsystems.ArmSubsystem arm;
    private pedroPathing.subsystems.ElevatorSubsystem elevator;

    private boolean lbPreviouslyPressed = false;

    @Override
    public void init() {
        // Inicializar o Follower para o drivetrain
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        // follower.setStartingPose(...); // Se necessário para telemetria de campo ou funcionalidades híbridas

        // Inicializar os subsistemas
        claw = new ClawSubsystem(hardwareMap);
        arm = new ArmSubsystem(hardwareMap);
        elevator = new ElevatorSubsystem(hardwareMap);

        telemetry.addData("Status", "TeleOp Principal Inicializado");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive(); // Coloca o drivetrain em modo teleoperado
    }

    @Override
    public void loop() {
        // --- Controle do Drivetrain (Gamepad 1) ---
        // Movimento padrão do Pedro Pathing
        double drive = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;
        // O último booleano é para field centric, geralmente false para robot centric no TeleOp
        follower.setTeleOpMovementVectors(drive, strafe, turn, false);
        follower.update(); // Atualiza o follower do Pedro Pathing

        // --- Controle da Garra (Gamepad 1) ---
        if (gamepad1.left_bumper &&!lbPreviouslyPressed) {
            claw.toggleClaw();
        }
        lbPreviouslyPressed = gamepad1.left_bumper;

        // --- Controle do Braço e Pulso (Gamepad 2) ---
        if (gamepad2.dpad_up) {
            arm.setArmPositionUp();
        } else if (gamepad2.dpad_down) {
            arm.setArmPositionDown();
        }

        // A lógica original para gamepad2.y e gamepad2.x era um pouco diferente para wrist2.
        // Mantendo a lógica original, mas agora através de métodos do subsistema.
        if (gamepad2.y) { // Originalmente movia ambos os pulsos para cima
            arm.setWrist1PositionUp();
            arm.setWrist2PositionUp(); // WRIST2_UP_POS
        } else if (gamepad2.x) { // Originalmente movia wrist1 para cima e wrist2 para uma posição específica (WRIST1_UP_POS)
            arm.setWrist1PositionUp();
            arm.setWrist2PositionUp(); // Se a intenção era WRIST1_UP_POS para ambos, então é isso.
            // Se era WRIST2_DOWN_POS ou outra, ajuste aqui.
            // Ex: arm.setBothWristsCustom(ArmSubsystem.WRIST1_UP_POS, ArmSubsystem.WRIST1_UP_POS);
            // Ou, se era para uma posição específica de "descanso" ou "coleta":
            // arm.setWrist2Position(AlgumaOutraPosicaoDefinidaNoSubsistema);
        }
        // Se você quiser controles separados para wrist1 e wrist2:
        // if (gamepad2.left_stick_button) arm.setWrist1PositionUp();
        // if (gamepad2.right_stick_button) arm.setWrist1PositionDown();


        // --- Controle do Elevador (Gamepad 2) ---
        // Controle Manual
        if (Math.abs(gamepad2.right_stick_y) > 0.1 &&!elevator.isMovingToPreset()) {
            // Usar o joystick direito do gamepad2 para controle manual do elevador
            // Negativo para subir, positivo para descer (padrão do joystick)
            elevator.setManualPower(-gamepad2.right_stick_y * ElevatorSubsystem.ELEVATOR_MANUAL_SPEED);
        } else if (!elevator.isMovingToPreset()) { // Se não houver entrada manual e não estiver em preset
            elevator.setManualPower(0); // Parar o motor se não houver comando manual
        }

        // Presets do Elevador com PID
        if (gamepad2.a) {
            elevator.goToPositionPID(ElevatorSubsystem.ELEVATOR_PRESET_LOW);
        } else if (gamepad2.b) {
            elevator.goToPositionPID(ElevatorSubsystem.ELEVATOR_PRESET_MEDIUM);
        } else if (gamepad2.right_bumper) { // Mudei do gamepad2.y para não conflitar com o pulso
            elevator.goToPositionPID(ElevatorSubsystem.ELEVATOR_PRESET_HIGH);
        } else if (gamepad2.dpad_left) { // Exemplo para ir para o chão
            elevator.goToPositionPID(ElevatorSubsystem.ELEVATOR_PRESET_GROUND);
        }


        // Atualizar o subsistema do elevador (para processar o PID)
        elevator.update();

        // --- Telemetria ---
        telemetry.addData("Drivetrain Pose", follower.getPose().toString());
        telemetry.addData("Claw", claw.isClawOpen()? "ABERTO" : "FECHADO");
        telemetry.addData("Claw Pos", "%.2f", claw.getClawPosition());
        telemetry.addData("Arm1 Pos", "%.2f", arm.getArm1Position());
        telemetry.addData("Arm2 Pos", "%.2f", arm.getArm2Position());
        telemetry.addData("Wrist1 Pos", "%.2f", arm.getWrist1Position());
        telemetry.addData("Wrist2 Pos", "%.2f", arm.getWrist2Position());
        telemetry.addData("Elevator Target Ticks", elevator.getTargetPosition());
        telemetry.addData("Elevator Current Ticks", elevator.getCurrentPosition());
        telemetry.addData("Elevator MovingToPreset", elevator.isMovingToPreset());
        telemetry.update();
    }
}