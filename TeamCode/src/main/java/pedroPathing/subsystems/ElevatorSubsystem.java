// Em pedroPathing.subsystems.ElevatorSubsystem.java
package pedroPathing.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
// REMOVIDO: import pedroPathing.examples.util.RobotGlobalState;

@Config
public class ElevatorSubsystem {
    private DcMotorEx elevatorMotor;

    //... (Suas constantes existentes: MAX_HEIGHT, PRESETS, PID, etc.)
    public static final int ELEVATOR_MAX_HEIGHT_TICKS = 3000;
    public static final double ELEVATOR_MANUAL_SPEED = 0.9;
    public static final int ELEVATOR_PRESET_LOW = 500;
    public static final int ELEVATOR_PRESET_MEDIUM = 1500;
    public static final int ELEVATOR_PRESET_HIGH = ELEVATOR_MAX_HEIGHT_TICKS;
    public static final int ELEVATOR_PRESET_GROUND = 0;
    public static double PID_P = 0.01;
    public static double PID_I = 0.001;
    public static double PID_D = 0.001;
    public static double PID_F = 0.12; // VALOR BASE! Preciso ajeitar

    private int targetPositionTicks;
    private boolean isPidActive = false;

    private ElapsedTime pidTimer = new ElapsedTime();
    private double lastError = 0;
    private double integralSum = 0;
    private static final double MAX_INTEGRAL_SUM = 1000;
    private static final double PID_OUTPUT_LIMIT = 0.8;

    public ElevatorSubsystem(HardwareMap hardwareMap) {
        elevatorMotor = hardwareMap.get(DcMotorEx.class, "elevatorMotor");
        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reseta para 0
        elevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        targetPositionTicks = 0;
    }

    public void goToPositionPID(int targetTicks) {
        this.targetPositionTicks = Math.max(0, Math.min(targetTicks, ELEVATOR_MAX_HEIGHT_TICKS));
        this.isPidActive = true;
        this.integralSum = 0;
        this.lastError = 0;
        this.pidTimer.reset();
    }

    public void setManualPower(double power) {
        if (Math.abs(power) > 0.05) {
            isPidActive = false;
        }
        if (isPidActive) {
            return;
        }
        int currentPosition = elevatorMotor.getCurrentPosition();
        if (power > 0 && currentPosition >= ELEVATOR_MAX_HEIGHT_TICKS) {
            elevatorMotor.setPower(0);
        } else if (power < 0 && currentPosition <= 0) {
            elevatorMotor.setPower(0);
        } else {
            elevatorMotor.setPower(power * ELEVATOR_MANUAL_SPEED);
        }
    }

    public void update() {
        if (!isPidActive) {
            return;
        }
        int currentPosition = elevatorMotor.getCurrentPosition();
        double error = targetPositionTicks - currentPosition;
        double deltaTime = pidTimer.seconds();
        pidTimer.reset();
        double proportional = PID_P * error;
        integralSum += error * deltaTime;
        integralSum = Math.max(-MAX_INTEGRAL_SUM, Math.min(integralSum, MAX_INTEGRAL_SUM));
        double integral = PID_I * integralSum;
        double derivative = 0;
        if (deltaTime > 0) {
            derivative = PID_D * (error - lastError) / deltaTime;
        }
        lastError = error;
        double feedforward = PID_F;
        double outputPower = proportional + integral + derivative + feedforward;
        outputPower = Math.max(-PID_OUTPUT_LIMIT, Math.min(outputPower, PID_OUTPUT_LIMIT));
        elevatorMotor.setPower(outputPower);
    }

    public void stopMotor() {
        elevatorMotor.setPower(0);
        isPidActive = false;
    }

    public int getCurrentPosition() { return elevatorMotor.getCurrentPosition(); }
    public int getTargetPosition() { return targetPositionTicks; }
    public boolean isMovingToPreset() { return isPidActive; }
}