package techmaker.subsystems;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import techmaker.constants.Constants;

@Config
public class ElevatorSubsystem {

    // --- Motores ---
    private final DcMotorEx LeftElevator;
    private final DcMotorEx RightElevator;

    // --- Constantes de Controle do Elevador ---
    public static int ELEVATOR_MAX_HEIGHT_TICKS = 3000;
    public static double ELEVATOR_POWER = 1.0;
    public static double ELEVATOR_DOWN_POWER_SCALE = -0.5;

    // --- Posições Predefinidas ---
    public static int ELEVATOR_PRESET_GROUND = 10;
    public static int ELEVATOR_PRESET_LOW = 500;
    public static int ELEVATOR_PRESET_MEDIUM = 1000;
    public static int ELEVATOR_PRESET_HIGH = 2000;

    // --- Constantes de Tuning do PID ---
    public static double PID_P = 0.015;
    public static double PID_I = 0.0;
    public static double PID_D = 0.0001;
    public static double GRAVITY_FF = 0.08;

    // --- Variáveis de Estado do PID ---
    private int targetPositionTicks;
    private boolean isPidActive = false;
    private final ElapsedTime pidTimer = new ElapsedTime();
    private double lastError = 0;
    private double integralSum = 0;

    public ElevatorSubsystem(@NonNull HardwareMap hardwareMap) {
        LeftElevator = hardwareMap.get(DcMotorEx.class, Constants.Elevator.LeftElevator);
        RightElevator = hardwareMap.get(DcMotorEx.class, Constants.Elevator.RightElevator);

        RightElevator.setDirection(DcMotorSimple.Direction.REVERSE);

        LeftElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LeftElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        targetPositionTicks = 0;
    }

    // --- Controle PID ---
    public void goToPositionPID(int targetTicks) {
        this.targetPositionTicks = Math.max(0, Math.min(targetTicks, ELEVATOR_MAX_HEIGHT_TICKS));
        this.isPidActive = true;
        this.integralSum = 0;
        this.lastError = 0;
        this.pidTimer.reset();
    }

    public void setManualPower(double power) {
        if (Math.abs(power) > 0.05) isPidActive = false;
        if (isPidActive) return;

        double motorPower = power * ELEVATOR_POWER;
        int currentPosition = getCurrentPosition();

        if (motorPower > 0 && currentPosition >= ELEVATOR_MAX_HEIGHT_TICKS) {
            motorPower = 0;
        } else if (motorPower < 0 && currentPosition <= 0) {
            motorPower = 0;
        }

        if (motorPower < 0) motorPower *= ELEVATOR_DOWN_POWER_SCALE;

        LeftElevator.setPower(motorPower);
        RightElevator.setPower(motorPower);
    }

    public int getCurrentPosition() {
        return (LeftElevator.getCurrentPosition() + RightElevator.getCurrentPosition()) / 2;
    }

    public void update(Telemetry telemetry) {
        if (isPidActive) {
            int currentPosition = getCurrentPosition();
            double error = targetPositionTicks - currentPosition;
            double deltaTime = pidTimer.seconds();
            pidTimer.reset();

            double proportional = PID_P * error;
            integralSum += error * deltaTime;
            double integral = PID_I * integralSum;

            double derivative = (deltaTime > 0) ? PID_D * (error - lastError) / deltaTime : 0;
            lastError = error;

            double feedforward = (targetPositionTicks > ELEVATOR_PRESET_GROUND) ? GRAVITY_FF : 0;
            double outputPower = proportional + integral + derivative + feedforward;

            LeftElevator.setPower(outputPower);
            RightElevator.setPower(outputPower);
        }

        // Opcional: Telemetria
        // sendTelemetry(telemetry);
    }

    // --- Função simples para verificar se chegou ---
    public boolean atTargetPosition(int tolerance) {
        return Math.abs(getCurrentPosition() - targetPositionTicks) <= tolerance;
    }

    private void sendTelemetry(Telemetry telemetry) {
        telemetry.addData("--- Elevador ---", "");
        telemetry.addData("PID Ativo", isPidActive);
        telemetry.addData("Posição Alvo", targetPositionTicks);
        telemetry.addData("Posição Atual", getCurrentPosition());
        telemetry.addData("Potência (L)", "%.2f", LeftElevator.getPower());
        telemetry.addData("Potência (R)", "%.2f", RightElevator.getPower());
        telemetry.addData("Corrente (L)", "%.2f A", LeftElevator.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Corrente (R)", "%.2f A", RightElevator.getCurrent(CurrentUnit.AMPS));
    }
}
