package techmaker.subsystems;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.concurrent.TimeUnit;

import techmaker.constants.Constants;

@Config
public class ElevatorSubsystem extends SubsystemBase {

    // --- Motores ---
    private final DcMotorEx LeftElevator;
    private final DcMotorEx RightElevator;

    // --- Constantes de Controle do Elevador ---
    public static int ELEVATOR_MAX_HEIGHT_TICKS = 3000;
    public static double ELEVATOR_POWER = 1.0;
    public static double ELEVATOR_DOWN_POWER_SCALE = -0.5; // Fator de escala para a descida manual

    // --- Posições Predefinidas ---
    public static int ELEVATOR_PRESET_GROUND = 10;
    public static int ELEVATOR_PRESET_LOW = 500;
    public static int ELEVATOR_PRESET_MEDIUM = 1000;
    public static int ELEVATOR_PRESET_HIGH = 2000;

    // --- Constantes de Tuning do PID ---
    public static double PID_P = 0.015;
    public static double PID_I = 0.0;
    public static double PID_D = 0.0001;
    public static double GRAVITY_FF = 0.08; // Feedforward para compensar a gravidade

    // --- Variáveis de Estado do PID ---
    private int targetPositionTicks;
    private boolean isPidActive = false;
    private final ElapsedTime pidTimer = new ElapsedTime();
    private double lastError = 0;
    private double integralSum = 0;

    /**
     * Construtor da classe do subsistema do elevador.
     * @param hardwareMap O mapa de hardware do robô.
     */
    public ElevatorSubsystem(@NonNull HardwareMap hardwareMap) {
        // Inicialização dos motores
        LeftElevator = hardwareMap.get(DcMotorEx.class, Constants.Elevator.LeftElevator);
        RightElevator = hardwareMap.get(DcMotorEx.class, Constants.Elevator.RightElevator);

        // Configuração dos motores
        RightElevator.setDirection(DcMotorSimple.Direction.REVERSE);

        LeftElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LeftElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Inicializa a posição alvo no chão
        targetPositionTicks = 0;
    }

    /**
     * Move o elevador para uma posição específica usando o controle PID.
     * @param targetTicks A posição alvo em ticks do encoder.
     */
    public void goToPositionPID(int targetTicks) {
        this.targetPositionTicks = Math.max(0, Math.min(targetTicks, ELEVATOR_MAX_HEIGHT_TICKS));
        this.isPidActive = true;
        this.integralSum = 0; // Reseta o somatório do erro integral
        this.lastError = 0;
        this.pidTimer.reset();
    }

    /**
     * Define a potência manual dos motores do elevador, desativando o PID.
     * @param power A potência a ser aplicada (-1.0 a 1.0).
     */
    public void setManualPower(double power) {
        // Se uma potência significativa for aplicada, desativa o modo PID
        if (Math.abs(power) > 0.05) {
            isPidActive = false;
        }

        // Não permite controle manual se o PID estiver ativo
        if (isPidActive) {
            return;
        }

        double motorPower = power * ELEVATOR_POWER;
        int currentPosition = getCurrentPosition();

        // Limita o movimento para não ultrapassar os limites físicos
        if (motorPower > 0 && currentPosition >= ELEVATOR_MAX_HEIGHT_TICKS) {
            motorPower = 0; // Impede de subir além do máximo
        } else if (motorPower < 0 && currentPosition <= 0) {
            motorPower = 0; // Impede de descer abaixo do mínimo
        }

        // Aplica uma escala de potência para a descida para um controle mais fino
        if (motorPower < 0) {
            motorPower *= ELEVATOR_DOWN_POWER_SCALE;
        }

        LeftElevator.setPower(motorPower);
        RightElevator.setPower(motorPower);
    }

    /**
     * Retorna a posição média atual dos encoders dos motores do elevador.
     * @return A posição média em ticks.
     */
    public int getCurrentPosition() {
        // Usar a média das posições pode fornecer uma leitura mais estável
        return (LeftElevator.getCurrentPosition() + RightElevator.getCurrentPosition()) / 2;
    }

    /**
     * O método de atualização principal a ser chamado em um loop no OpMode.
     * Executa o cálculo do PID se estiver ativo e envia dados para a telemetria.
     * @param telemetry O objeto de telemetria para exibir informações.
     */
    public void update(Telemetry telemetry) {
        // Atualiza o controle PID se estiver ativo
        if (isPidActive) {
            int currentPosition = getCurrentPosition();
            double error = targetPositionTicks - currentPosition;
            double deltaTime = pidTimer.seconds();
            pidTimer.reset();

            // Termo Proporcional
            double proportional = PID_P * error;

            // Termo Integral com anti-windup (limita o somatório)
            integralSum += error * deltaTime;
            // Opcional: Adicionar um limite para integralSum para evitar wind-up
            // integralSum = Math.max(-integralMax, Math.min(integralSum, integralMax));
            double integral = PID_I * integralSum;

            // Termo Derivativo
            double derivative = 0;
            if (deltaTime > 0) {
                derivative = PID_D * (error - lastError) / deltaTime;
            }
            lastError = error;

            // Termo Feedforward para compensar a gravidade
            // Aplica a força somente quando o elevador precisa se sustentar contra a gravidade
            double feedforward = (targetPositionTicks > ELEVATOR_PRESET_GROUND) ? GRAVITY_FF : 0;

            // Potência final de saída
            double outputPower = proportional + integral + derivative + feedforward;

            // Aplica a potência aos motores
            LeftElevator.setPower(outputPower);
            RightElevator.setPower(outputPower);
        }

        // Atualiza a telemetria
        //sendTelemetry(telemetry);
    }

    /**
     * Envia dados de telemetria do elevador.
     * @param telemetry O objeto de telemetria.
     */
    private void sendTelemetry(Telemetry telemetry) {
        telemetry.addData("--- Elevador ---", "");
        telemetry.addData("Modo PID Ativo", isPidActive);
        telemetry.addData("Posição Alvo (Ticks)", targetPositionTicks);
        telemetry.addData("Posição Atual (Ticks)", getCurrentPosition());
        telemetry.addData("Potência (L)", "%.2f", LeftElevator.getPower());
        telemetry.addData("Potência (R)", "%.2f", RightElevator.getPower());
        telemetry.addData("Corrente (L)", "%.2f A", LeftElevator.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Corrente (R)", "%.2f A", RightElevator.getCurrent(CurrentUnit.AMPS));
    }
}