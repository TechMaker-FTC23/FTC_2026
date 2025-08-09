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
    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;

    public static int ELEVATOR_MAX_HEIGHT_TICKS = 3000;
    public static double ELEVATOR_POWER = 1.0;
    public static double ELEVATOR_DOWN_POWER = 0.1;

    public static int ELEVATOR_PRESET_LOW = 500;
    public static int ELEVATOR_PRESET_MEDIUM = 2000;
    public static int ELEVATOR_PRESET_HIGH = 2800;
    public static int ELEVATOR_PRESET_GROUND = 10;

    // --- Constantes de Tuning do PID ---
    public static double PID_P = 0.008;
    public static double PID_I = 0.0;
    public static double PID_D = 0.0001;
    public static double GRAVITY_FF = 0.08; // Feedforward para compensar a gravidade.

    public ElevatorSubsystem(@NonNull HardwareMap hardwareMap) {
        try {
            leftMotor = hardwareMap.get(DcMotorEx.class, Constants.Elevator.LeftElevator);
            rightMotor = hardwareMap.get(DcMotorEx.class, Constants.Elevator.RightElevator);

            rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // MUDANÇA: O GRAVITY_FF agora é passado como o parâmetro 'F' do PIDF.
            // Esta é a maneira correta de usar o feedforward com o controlador do motor.
            PIDFCoefficients pidfCoefficients = new PIDFCoefficients(PID_P, PID_I, PID_D, GRAVITY_FF);
            leftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfCoefficients);
            rightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfCoefficients);

            resetEncoders();
        } catch (Exception e) {
            leftMotor = null;
            rightMotor = null;
        }
    }

    public void goToPosition(int targetTicks) {
        if (leftMotor == null || rightMotor == null) return;

        int clampedTarget = Math.max(0, Math.min(targetTicks, ELEVATOR_MAX_HEIGHT_TICKS));

        leftMotor.setTargetPosition(clampedTarget);
        rightMotor.setTargetPosition(clampedTarget);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double power;
        if (clampedTarget < getCurrentPosition()) {
            power = ELEVATOR_DOWN_POWER;
        } else {
            power = ELEVATOR_POWER;
        }

        // MUDANÇA: Não adicionamos mais o GRAVITY_FF aqui. O controlador interno do
        // motor já está a tratar disso através do coeficiente F.
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    public void holdPosition() {
        if (leftMotor == null || rightMotor == null) return;

        goToPosition(getCurrentPosition());
    }

    public void stopMotors() {
        if (leftMotor == null || rightMotor == null) return;

        leftMotor.setPower(0);
        rightMotor.setPower(0);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public boolean isBusy() {
        if (leftMotor == null || rightMotor == null) return false;
        return leftMotor.isBusy() || rightMotor.isBusy();
    }

    public void resetEncoders() {
        if (leftMotor == null || rightMotor == null) return;
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public int getCurrentPosition() {
        if (leftMotor == null || rightMotor == null) return 0;
        return (leftMotor.getCurrentPosition() + rightMotor.getCurrentPosition()) / 2;
    }

    public void update(Telemetry telemetry) {
        if (leftMotor == null || rightMotor == null) {
            telemetry.addData("Elevador", "NÃO CONECTADO");
            return;
        }

        telemetry.addData("Elevador Alvo", leftMotor.getTargetPosition());
        telemetry.addData("Elevador Posição Atual", getCurrentPosition());
        telemetry.addData("Elevador Potência (L)", "%.2f", leftMotor.getPower());
        telemetry.addData("Elevador Corrente (L)", "%.2f A", leftMotor.getCurrent(CurrentUnit.AMPS));
    }
}
