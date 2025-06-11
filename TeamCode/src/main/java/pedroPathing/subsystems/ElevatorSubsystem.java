package pedroPathing.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ElevatorSubsystem {
    private DcMotorEx elevatorMotor;

    // Configuração do Elevador
    public static final int ELEVATOR_MAX_HEIGHT_TICKS = 3000;
    public static final double ELEVATOR_MANUAL_SPEED = 0.3; // Mantido para controle manual

    // Presets do Elevador
    public static final int ELEVATOR_PRESET_LOW = 500;
    public static final int ELEVATOR_PRESET_MEDIUM = 1500;
    public static final int ELEVATOR_PRESET_HIGH = ELEVATOR_MAX_HEIGHT_TICKS;
    public static final int ELEVATOR_PRESET_GROUND = 0; // Adicionado para referência

    // --- Configurações do PID ---
    // Estes valores PRECISAM ser ajustados (tuned) para o seu robô específico!
    public static double PID_P = 0.005;
    public static double PID_I = 0.0001;
    public static double PID_D = 0.0002;
    public static double PID_F = 0.0;    // Exemplo: Ganho Feedforward (pode ser usado para combater gravidade)
    // Se o elevador tende a cair, um F positivo pequeno pode ajudar.

    private int targetPositionTicks;
    private boolean movingToPresetPID = false;

    // Variáveis para o cálculo do PID
    private ElapsedTime pidTimer = new ElapsedTime();
    private double lastError = 0;
    private double integralSum = 0;
    private static final double MAX_INTEGRAL_SUM = 1000; // Limite para anti-windup
    private static final double PID_OUTPUT_LIMIT = 0.7; // Limita a potência máxima do PID

    public ElevatorSubsystem(HardwareMap hardwareMap) {
        elevatorMotor = hardwareMap.get(DcMotorEx.class, "elevatorMotor");
        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Usaremos RUN_WITHOUT_ENCODER para controle PID manual,
        // ou RUN_USING_ENCODER se quisermos usar o PID interno do motor para controle manual.
        // Para nosso PID externo, RUN_WITHOUT_ENCODER é mais direto.
        elevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        targetPositionTicks = 0; // Posição inicial
    }

    // Método para mover para uma posição usando PID
    public void goToPositionPID(int targetTicks) {
        this.targetPositionTicks = Math.max(0, Math.min(targetTicks, ELEVATOR_MAX_HEIGHT_TICKS));
        this.movingToPresetPID = true;
        // Resetar o estado do PID para um novo movimento
        this.integralSum = 0;
        this.lastError = 0;
        this.pidTimer.reset();
    }

    // Controle manual do elevador (sem PID, potência direta)
    public void setManualPower(double power) {
        if (movingToPresetPID) { // Se estiver em modo PID, não permitir controle manual direto
            return;
        }
        // Aplicar limites de segurança para controle manual
        int currentPosition = elevatorMotor.getCurrentPosition();
        if (power > 0 && currentPosition >= ELEVATOR_MAX_HEIGHT_TICKS) {
            elevatorMotor.setPower(0);
        } else if (power < 0 && currentPosition <= 0) {
            elevatorMotor.setPower(0);
        } else {
            elevatorMotor.setPower(power);
        }
    }

    // Deve ser chamado repetidamente no loop do OpMode
    public void update() {
        if (movingToPresetPID) {
            int currentPosition = elevatorMotor.getCurrentPosition();
            double error = targetPositionTicks - currentPosition;
            double deltaTime = pidTimer.seconds();
            pidTimer.reset();

            // Termo Proporcional
            double proportional = PID_P * error;

            // Termo Integral (com anti-windup)
            integralSum += error * deltaTime;
            integralSum = Math.max(-MAX_INTEGRAL_SUM, Math.min(integralSum, MAX_INTEGRAL_SUM));
            double integral = PID_I * integralSum;

            // Termo Derivativo
            double derivative = 0;
            if (deltaTime > 0) {
                derivative = PID_D * (error - lastError) / deltaTime;
            }
            lastError = error;

            // Termo Feedforward (exemplo simples para manter a posição contra a gravidade)
            // Você pode precisar de um valor de F diferente se o elevador estiver subindo vs descendo,
            // ou um F que dependa da posição (mais complexo).
            // Um F constante pode ajudar a manter a posição.
            double feedforward = PID_F; // Ajuste este valor

            double outputPower = proportional + integral + derivative + feedforward;

            // Limitar a saída do PID
            outputPower = Math.max(-PID_OUTPUT_LIMIT, Math.min(outputPower, PID_OUTPUT_LIMIT));

            elevatorMotor.setPower(outputPower);

            // Condição para parar o PID (ex: dentro de uma tolerância)
            if (Math.abs(error) < 20) { // Tolerância de 20 ticks, ajuste conforme necessário
                // Opcional: manter uma pequena potência para segurar a posição se F não for suficiente
                // elevatorMotor.setPower(PID_F); // ou 0 se o freio for suficiente
                movingToPresetPID = false; // Considerar se deve parar ou manter com F
            }
        }
        // Se não estiver em movingToPresetPID, a potência é controlada por setManualPower
    }

    public void stopMotor() {
        elevatorMotor.setPower(0);
        movingToPresetPID = false;
    }

    public int getCurrentPosition() {
        return elevatorMotor.getCurrentPosition();
    }

    public int getTargetPosition() {
        return targetPositionTicks;
    }

    public boolean isMovingToPreset() {
        return movingToPresetPID;
    }
}