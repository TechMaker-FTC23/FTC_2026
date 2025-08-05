package techmaker.subsystems;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.concurrent.TimeUnit;

@Config
public class ElevatorSubsystem {
    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;

    public static int ELEVATOR_MAX_HEIGHT_TICKS = 3000;
    public static double ELEVATOR_MANUAL_SPEED = 1.0;
    public static double ELEVATOR_DOWN_SCALE = 0.2;
    public static int ELEVATOR_PRESET_LOW = 500;
    public static int ELEVATOR_PRESET_MEDIUM = 2000;
    public static int ELEVATOR_PRESET_HIGH = 2800;
    public static int ELEVATOR_PRESET_GROUND = -50;

    public static double PID_P = 0.01;
    public static double PID_I = 0.0;
    public static double PID_D = 0.0;
    public static double PID_F = 1.0;

    private int targetPositionTicks;
    private boolean isPidActive = false;
    private final ElapsedTime pidTimer = new ElapsedTime();
    private double lastError = 0;
    private double integralSum = 0;
    private ElapsedTime timer = new ElapsedTime();

    public ElevatorSubsystem(@NonNull HardwareMap hardwareMap) {
        leftMotor = hardwareMap.get(DcMotorEx.class, "leftElevatorMotor");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightElevatorMotor");
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        targetPositionTicks = 0;
        timer.reset();
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
        int currentPosition = leftMotor.getCurrentPosition();
        double motorPower = power * ELEVATOR_MANUAL_SPEED;

        if (motorPower < 0) {
            motorPower *= ELEVATOR_DOWN_SCALE;
        }

        if (motorPower > 0 && currentPosition >= ELEVATOR_MAX_HEIGHT_TICKS) {
            motorPower = 0;
        } else if (motorPower < 0 && currentPosition <= 0) {
            motorPower = 0;
        }

        leftMotor.setPower(motorPower);
        rightMotor.setPower(motorPower);
    }

    public void update(Telemetry telemetry) {
        if (timer.time(TimeUnit.MILLISECONDS) > 20) {
            timer.reset();

            if (!isPidActive) {
                return;
            }
            int currentPosition = getCurrentPosition();
            double error = targetPositionTicks - currentPosition;
            double deltaTime = pidTimer.seconds();
            pidTimer.reset();
            double proportional = PID_P * error;
            integralSum += error * deltaTime;
            double integral = PID_I * integralSum;
            double derivative = 0;
            if (deltaTime > 0) {
                derivative = PID_D * (error - lastError) / deltaTime;
            }
            lastError = error;
            double feedforward = (targetPositionTicks > 5) ? PID_F : 0;
            double outputPower = proportional + integral + derivative + feedforward;
            leftMotor.setPower(outputPower);
            rightMotor.setPower(outputPower);
        }
    }

    public void stopMotor() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        isPidActive = false;
    }

    public int getCurrentPosition() {
        return (leftMotor.getCurrentPosition() + rightMotor.getCurrentPosition()) / 2;
    }

    public int getTargetPosition() {
        return targetPositionTicks;
    }

    public boolean isMovingToPreset() {
        return isPidActive;
    }
}
