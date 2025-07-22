package pedroPathing.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

// ALTERADO: O nome da classe agora descreve melhor sua função completa.
public class ClawSubsystem {

    // --- Constantes ---

    // NOVO: Constantes para os nomes do hardware. Evita erros de digitação e facilita a manutenção.
    public static final String LEFT_INTAKE_NAME = "leftIntake";
    public static final String RIGHT_INTAKE_NAME = "rightIntake";
    public static final String LEFT_CLAW_SERVO_NAME = "leftClawServo";
    public static final String RIGHT_CLAW_SERVO_NAME = "rightClawServo";

    // Constante de potência para o intake
    public static final double INTAKE_POWER = 1.0;

    // --- Declarações de Hardware ---
    private CRServo leftIntake;
    private CRServo rightIntake;
    private Servo leftClawServo;
    private Servo rightClawServo;

    /**
     * Construtor do subsistema unificado do efetor final.
     * @param hardwareMap O mapa de hardware do robô.
     */
    public ClawSubsystem(HardwareMap hardwareMap) {
        // Mapeamento do hardware usando as constantes
        leftIntake = hardwareMap.get(CRServo.class, LEFT_INTAKE_NAME);
        rightIntake = hardwareMap.get(CRServo.class, RIGHT_INTAKE_NAME);
        leftClawServo = hardwareMap.get(Servo.class, LEFT_CLAW_SERVO_NAME);
        rightClawServo = hardwareMap.get(Servo.class, RIGHT_CLAW_SERVO_NAME);

        // Configuração inicial dos mecanismos
        rightClawServo.setDirection(Servo.Direction.REVERSE); // Essencial para a garra diferencial
        stopIntake(); // Boa prática: garantir que o intake comece parado
    }

    // --- MÉTODOS DE CONTROLE DO INTAKE ---

    /**
     * Aciona os roletes para puxar objetos para dentro do robô.
     */
    public void runIntake() {
        // Lembre-se de testar: talvez ambos os lados precisem de potência positiva
        // dependendo da montagem física.
        leftIntake.setPower(INTAKE_POWER);
        rightIntake.setPower(-INTAKE_POWER);
    }

    /**
     * Aciona os roletes para ejetar objetos.
     */
    public void runOuttake() {
        leftIntake.setPower(-INTAKE_POWER);
        rightIntake.setPower(INTAKE_POWER);
    }

    /**
     * Para completamente os roletes do intake.
     */
    public void stopIntake() {
        leftIntake.setPower(0);
        rightIntake.setPower(0);
    }

    // --- MÉTODOS DE CONTROLE DA GARRA DIFERENCIAL ---

    /**
     * Controla a garra com precisão usando valores de grip (pegar) e rotação.
     * Ideal para ser usado com joysticks.
     * @param grip Valor de -1.0 (totalmente aberto) a 1.0 (totalmente fechado).
     * @param rotation Valor de -1.0 (rotacionar max esquerda) a 1.0 (rotacionar max direita).
     */
    public void controlClaw(double grip, double rotation) {
        // 1. Mixagem dos inputs
        double leftPower = rotation + grip;
        double rightPower = rotation - grip;

        // 2. Normalização
        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (max > 1.0) {
            leftPower /= max;
            rightPower /= max;
        }

        // 3. Mapeamento para o intervalo do Servo [0, 1]
        double leftPosition = (leftPower + 1) / 2.0;
        double rightPosition = (rightPower + 1) / 2.0;

        // 4. Define a posição dos servos
        leftClawServo.setPosition(leftPosition);
        rightClawServo.setPosition(rightPosition);
    }

    // NOVO: Métodos de conveniência para ações rápidas e para o modo autônomo.

    /**
     * Fecha completamente a garra, mantendo a rotação centralizada.
     */
    public void closeClaw() {
        controlClaw(1.0, 0.0);
    }

    /**
     * Abre completamente a garra, mantendo a rotação centralizada.
     */
    public void openClaw() {
        controlClaw(-1.0, 0.0);
    }

    /**
     * Move a garra para uma posição de coleta predefinida (semiaberta e centralizada).
     * O valor de grip (-0.2) pode ser ajustado para o tamanho da peça do jogo.
     */
    public void goToCollectionPose() {
        controlClaw(-0.2, 0.0);
    }
}