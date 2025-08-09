package techmaker.subsystems;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import techmaker.constants.Constants;

@Config
public class ElevatorSubsystem {
    private DcMotorEx LeftElevator;
    private DcMotorEx RightElevator;

    public static int ELEVATOR_MAX_HEIGHT_TICKS = 3000;
    public static double ELEVATOR_POWER = 1.0;
    public static double ELEVATOR_DOWN_POWER = 0.1;

    public static int ELEVATOR_PRESET_LOW = 500;
    public static int ELEVATOR_PRESET_MEDIUM = 1000;
    public static int ELEVATOR_PRESET_HIGH = 2000;
    public static int ELEVATOR_PRESET_GROUND = 10;

    // --- Constantes de Tuning do PID ---
    public static double PID_P = 0.008;
    public static double PID_I = 0.0;
    public static double PID_D = 0.0001;
    public static double GRAVITY_FF = 0.08; // Feedforward para compensar a gravidade.

    public ElevatorSubsystem(@NonNull HardwareMap hardwareMap) {
        try {

            LeftElevator = hardwareMap.get(DcMotorEx.class, Constants.Elevator.LeftElevator);
            RightElevator = hardwareMap.get(DcMotorEx.class, Constants.Elevator.RightElevator);
            LeftElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RightElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            RightElevator.setDirection(DcMotorSimple.Direction.REVERSE);
            LeftElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RightElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            PIDFCoefficients pidfCoefficients = new PIDFCoefficients(PID_P, PID_I, PID_D, GRAVITY_FF);
            LeftElevator.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfCoefficients);
            RightElevator.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfCoefficients);

            resetEncoders();
        } catch (Exception e) {
            LeftElevator = null;
            RightElevator = null;
        }
    }

    public void goToPosition(int targetTicks) {
        if (LeftElevator == null || RightElevator == null) return;

        int clampedTarget = Math.max(0, Math.min(targetTicks, ELEVATOR_MAX_HEIGHT_TICKS));

        LeftElevator.setTargetPosition(clampedTarget);
        RightElevator.setTargetPosition(clampedTarget);

        LeftElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double power = (clampedTarget < getCurrentPosition()) ? ELEVATOR_DOWN_POWER : ELEVATOR_POWER;
        LeftElevator.setPower(power);
        RightElevator.setPower(power);
    }



    public void holdPosition() {
        if (LeftElevator == null || RightElevator == null) return;

        goToPosition(getCurrentPosition());
    }

    public void stopMotors() {
        if (LeftElevator == null || RightElevator == null) return;

        LeftElevator.setPower(0);
        RightElevator.setPower(0);

        LeftElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public boolean isBusy() {
        if (LeftElevator == null || RightElevator == null) return false;
        return LeftElevator.isBusy() || RightElevator.isBusy();
    }

    public void resetEncoders() {
        if (LeftElevator == null || RightElevator == null) return;
        LeftElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public int getCurrentPosition() {
        if (LeftElevator == null || RightElevator == null) return 0;
        return (LeftElevator.getCurrentPosition() + RightElevator.getCurrentPosition()) / 2;
    }

    public void update(Telemetry telemetry) {
        if (LeftElevator == null || RightElevator == null) {
            telemetry.addData("Elevador", "NÃO CONECTADO");
            return;
        }

        telemetry.addData("Elevador Alvo", LeftElevator.getTargetPosition());
        telemetry.addData("Elevador Posição Atual", getCurrentPosition());
        telemetry.addData("Elevador Potência (L)", "%.2f", LeftElevator.getPower());
        telemetry.addData("Elevador Corrente (L)", "%.2f A", LeftElevator.getCurrent(CurrentUnit.AMPS));
    }
}
