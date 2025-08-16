package techmaker.subsystems;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import techmaker.core.ftclib.command.SubsystemBase;
import techmaker.constants.Constants;

@Config
public class ClawSubsystem extends SubsystemBase {

    // Enum para representar os estados da garra de forma clara.
    public enum ClawState {
        INTAKE,  // Posição para pegar os pixels do intake
        TRAVEL,  // Posição segura para se movimentar pelo campo
        SCORE    // Posição para pontuar no backdrop
    }

    // --- Constantes de Posição com Nomes Clarificados ---
    // Posições para o estado INTAKE
    public static double ARM_LEFT_INTAKE_CLAW = 0.8;
    public static double ARM_RIGHT_INTAKE_CLAW = 0.2;

    public static double WRIST_LEFT_INTAKE_CLAW = 0.7;
    public static double WRIST_RIGHT_INTAKE_CLAW = 0.3;

    // Posições para o estado TRAVEL (intermediário)
    public static double ARM_LEFT_TRAVEL_CLAW = 0.7;
    public static double ARM_RIGHT_TRAVEL_CLAW = 0.3;
    public static double WRIST_LEFT_TRAVEL_CLAW = 0.8;
    public static double WRIST_RIGHT_TRAVEL_CLAW = 0.2;

    // Posições para o estado SCORE
    public static double ARM_LEFT_SCORE_CLAW = 0.10;
    public static double ARM_RIGHT_SCORE_CLAW = 0.9;
    public static double WRIST_LEFT_SCORE_CLAW = 0.6;
    public static double WRIST_RIGHT_SCORE_CLAW = 0.4;

    // Posições da Garra Central (aberta/fechada)
    public static double CLAW_OPEN = 0.4;
    public static double CLAW_CLOSED = 0.65;


    // --- Declarações de Hardware ---
    private Servo middleClawServo;
    private Servo leftClawWrist;
    private Servo rightClawWrist;
    private Servo leftClawArm;
    private Servo rightClawArm;
    private Telemetry telemetry;
    public ClawSubsystem(@NonNull HardwareMap hardwareMap,@NonNull Telemetry telemetry) {
        this.telemetry = telemetry;
        // Mapeamento robusto com try-catch
        try {
            middleClawServo = hardwareMap.get(Servo.class, Constants.Claw.MiddleClaw);
            leftClawWrist = hardwareMap.get(Servo.class, Constants.Claw.LeftClaw);
            rightClawWrist = hardwareMap.get(Servo.class, Constants.Claw.RightClaw);
            leftClawArm = hardwareMap.get(Servo.class, Constants.Claw.LeftArm);
            rightClawArm = hardwareMap.get(Servo.class, Constants.Claw.RightArm);
        } catch (Exception e) {
            middleClawServo = null;
            leftClawWrist = null;
            rightClawWrist = null;
            leftClawArm = null;
            rightClawArm = null;
        }
    }

    /**
     * Método principal para controlar a garra.
     * Move todos os servos para um estado pré-definido.
     * @param state O estado desejado (INTAKE, TRAVEL, SCORE).
     */
    public void setState(ClawState state) {
        switch (state) {
            case INTAKE:
                setArmPosition(ARM_LEFT_INTAKE_CLAW, ARM_RIGHT_INTAKE_CLAW);
                setWristPosition(WRIST_LEFT_INTAKE_CLAW, WRIST_RIGHT_INTAKE_CLAW);
                break;
            case TRAVEL:
                setArmPosition(ARM_LEFT_TRAVEL_CLAW, ARM_RIGHT_TRAVEL_CLAW);
                setWristPosition(WRIST_LEFT_TRAVEL_CLAW, WRIST_RIGHT_TRAVEL_CLAW);
                break;
            case SCORE:
                setArmPosition(ARM_LEFT_SCORE_CLAW, ARM_RIGHT_SCORE_CLAW);
                setWristPosition(WRIST_LEFT_SCORE_CLAW, WRIST_RIGHT_SCORE_CLAW);
                break;
        }
    }

    /**
     * Controla a garra central para abrir ou fechar.
     * @param open Se true, abre a garra; se false, fecha.
     */
    public void setClawOpen(boolean open) {
        if (middleClawServo != null) {
            if (open) {
                middleClawServo.setPosition(CLAW_OPEN);
            } else {
                middleClawServo.setPosition(CLAW_CLOSED);
            }
        }
    }

    // --- Métodos de baixo nível ---

    public void setArmPosition(double positionL, double positionR) {
        if (leftClawArm != null) leftClawArm.setPosition(positionL);
        if (rightClawArm != null) rightClawArm.setPosition(positionR);
    }

    public void setWristPosition(double positionL, double positionR) {
        if (leftClawWrist != null) leftClawWrist.setPosition(positionL);
        if (rightClawWrist != null) rightClawWrist.setPosition(positionR);
    }

    public void setMiddleClawPosition(double position) {
        if (middleClawServo != null) middleClawServo.setPosition(position);
    }

    public void periodic() {
        telemetry.addData("Claw Servo", (middleClawServo != null) ? "OK" : "NÃO CONECTADO");
        telemetry.addData("Wrist Servos", (leftClawWrist != null && rightClawWrist != null) ? "OK" : "NÃO CONECTADO");
        telemetry.addData("Arm Servos", (leftClawArm != null && rightClawArm != null) ? "OK" : "NÃO CONECTADO");
    }
}
