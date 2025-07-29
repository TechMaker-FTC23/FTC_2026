// Arquivo: techmaker/RobotMechanisms.java

package techmaker;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
// Removido o import do VoltageSensor

import com.pedropathing.follower.Follower;

import techmaker.constants.FConstants;
import techmaker.constants.LConstants;

import techmaker.subsystems.ClawSubsystem;
import techmaker.subsystems.ElevatorSubsystem;
import techmaker.subsystems.IntakeSubsystem;

@Config
@TeleOp(name = "Controle Principal do Robô", group = "PedroPathing")
public class RobotMechanisms extends OpMode {

    // --- Constantes de Tempo FIXAS ---
    public static double TEMPO_SLIDER_EXTEND = 0.7;
    public static double TEMPO_WRIST_RAISE = 0.5;
    public static double TEMPO_SLIDER_RETRACT = 0.8;

    // --- Subsistemas e Controle ---
    private Follower follower;
    private ClawSubsystem claw;
    private IntakeSubsystem intake;
    // Removida a variável do VoltageSensor
    private final Pose startPose = new Pose(0, 0, 0);

    // --- Máquina de Estados ---
    private enum EstadoColeta { PARADO, ESTENDENDO, COLETANDO, RECOLHENDO_INICIO, RECOLHENDO_FIM }
    private EstadoColeta estadoColeta = EstadoColeta.PARADO;
    private ElapsedTime coletaTimer = new ElapsedTime();

    // Variáveis para detectar um único toque de botão
    private boolean circleJaFoiPressionado = false;

    @Override
    public void init() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        claw = new ClawSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap, false);

        // Removido o mapeamento do sensor de voltagem

        // --- Posição Inicial da Garra ---
        claw.clawArm(ClawSubsystem.medArml, ClawSubsystem.medArmR);

        telemetry.addData("Status", "TeleOp Principal Inicializado");
        telemetry.addData("Garra", "Posição inicial definida para MED");
        telemetry.addData("Controles", "Circle para coletar, X para recolher");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        coletaTimer.reset();
    }

    @Override
    public void loop() {
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        follower.update();

        handleIntakeAutomation();
        handleClawControls();

        intake.update(telemetry);
        claw.update(telemetry);
        telemetry.addData("Estado do Intake", estadoColeta);
        // Removida a telemetria da voltagem
        telemetry.update();
    }

    private void handleIntakeAutomation() {
        // Removida a lógica do timer dinâmico

        boolean circlePressionadoAgora = gamepad1.circle;
        boolean xPressionadoAgora = gamepad1.cross;

        switch (estadoColeta) {
            case PARADO:
                if (circlePressionadoAgora && !circleJaFoiPressionado) {
                    estadoColeta = EstadoColeta.ESTENDENDO;
                    coletaTimer.reset();
                }
                break;

            case ESTENDENDO:
                intake.slider(IntakeSubsystem.LEFT_INTAKE_SLIDER_MAX, IntakeSubsystem.RIGHT_INTAKE_SLIDER_MAX);
                // A condição de tempo agora é fixa
                if (coletaTimer.seconds() > TEMPO_SLIDER_EXTEND) {
                    estadoColeta = EstadoColeta.COLETANDO;
                }
                if (xPressionadoAgora) {
                    estadoColeta = EstadoColeta.RECOLHENDO_INICIO;
                    coletaTimer.reset();
                }
                break;

            case COLETANDO:
                intake.wrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MAX, IntakeSubsystem.RIGHT_INTAKE_WRIST_MAX);
                intake.startIntake();
                if (xPressionadoAgora) {
                    estadoColeta = EstadoColeta.RECOLHENDO_INICIO;
                    coletaTimer.reset();
                }
                break;

            case RECOLHENDO_INICIO:
                intake.stopIntake();
                intake.wrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MIN, IntakeSubsystem.RIGHT_INTAKE_WRIST_MIN);
                if (coletaTimer.seconds() > TEMPO_WRIST_RAISE) {
                    estadoColeta = EstadoColeta.RECOLHENDO_FIM;
                }
                break;

            case RECOLHENDO_FIM:
                intake.slider(IntakeSubsystem.LEFT_INTAKE_SLIDER_MIN, IntakeSubsystem.RIGHT_INTAKE_SLIDER_MIN);
                if (coletaTimer.seconds() > TEMPO_SLIDER_RETRACT) {
                    estadoColeta = EstadoColeta.PARADO;
                }
                break;
        }
        circleJaFoiPressionado = circlePressionadoAgora;
    }

    private void handleClawControls() {
        if(gamepad1.dpad_down){
            claw.clawArm(ClawSubsystem.medArml,ClawSubsystem.medArmR);
        } else if (gamepad1.dpad_up){
            claw.clawArm(ClawSubsystem.maxArmL,ClawSubsystem.maxArmR);
        } else {
            claw.clawArm(ClawSubsystem.minArmL,ClawSubsystem.minArmR);
        }

        if(gamepad1.square){
            claw.clawWrist(ClawSubsystem.maxWristL,ClawSubsystem.maxWristR);
        } else {
            claw.clawWrist(ClawSubsystem.minWristL,ClawSubsystem.minWristR);
        }

        if(gamepad1.triangle){
            claw.middleClaw(ClawSubsystem.maxClaw);
        } else {
            claw.middleClaw(ClawSubsystem.minClaw);
        }
    }
}