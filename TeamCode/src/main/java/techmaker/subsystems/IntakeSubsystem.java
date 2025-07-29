// Arquivo: techmaker/subsystems/IntakeSubsystem.java

package techmaker.subsystems;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.concurrent.TimeUnit;

@Config
public class IntakeSubsystem {

    public static final String LEFT_INTAKE_NAME = "leftIntake";
    public static final String RIGHT_INTAKE_NAME = "rightIntake";
    public static final String LEFT_INTAKE_WRIST_NAME = "leftIntakeWrist";
    public static final String RIGHT_INTAKE_WRIST_NAME = "rightIntakeWrist";
    public static final String LEFT_INTAKE_SLIDER_NAME = "leftIntakeSlider";
    public static final String RIGHT_INTAKE_SLIDER_NAME = "rightIntakeSlider";

    public static double LEFT_INTAKE_WRIST_MAX = 1;
    public static double RIGHT_INTAKE_WRIST_MAX = 0;
    public static double LEFT_INTAKE_SLIDER_MAX = 0.75;
    public static double RIGHT_INTAKE_SLIDER_MAX = 0.25;
    public static double LEFT_INTAKE_WRIST_MIN = 0.3;
    public static double RIGHT_INTAKE_WRIST_MIN = 0.7;
    public static double LEFT_INTAKE_SLIDER_MIN = 1;
    public static double RIGHT_INTAKE_SLIDER_MIN = 0;

    public static final String COLOR_SENSOR_NAME = "colorSensor";

    private final CRServo leftIntake;
    private final CRServo rightIntake;
    private final Servo leftIntakeWrist;
    private final Servo rightIntakeWrist;
    private final Servo leftIntakeSlider;
    private final Servo rightIntakeSlider;
    private final ColorSensor colorSensor;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean isIntakeActive = false;
    private boolean isRedAlliance = false;

    public IntakeSubsystem(@NonNull HardwareMap hardwareMap, boolean isRedAlliance) {
        leftIntake = hardwareMap.get(CRServo.class, LEFT_INTAKE_NAME);
        rightIntake = hardwareMap.get(CRServo.class, RIGHT_INTAKE_NAME);
        leftIntakeWrist = hardwareMap.get(Servo.class, LEFT_INTAKE_WRIST_NAME);
        rightIntakeWrist = hardwareMap.get(Servo.class, RIGHT_INTAKE_WRIST_NAME);
        leftIntakeSlider = hardwareMap.get(Servo.class, LEFT_INTAKE_SLIDER_NAME);
        rightIntakeSlider = hardwareMap.get(Servo.class, RIGHT_INTAKE_SLIDER_NAME);

        // Lógica do sensor desativada com segurança
        colorSensor = null;

        this.isRedAlliance = isRedAlliance;
        leftIntake.setDirection(CRServo.Direction.REVERSE);
        timer.reset();
        stopIntake();
    }

    public void update(Telemetry telemetry) {
        if (timer.time(TimeUnit.MILLISECONDS) > 20) {
            timer.reset();
            if (colorSensor != null) { // Checagem para evitar erro
                telemetry.addData("Red", colorSensor.red());
                telemetry.addData("Blue", colorSensor.blue());
                telemetry.addData("Green", colorSensor.green());
            }
        }
    }

    public void startIntake() {
        isIntakeActive = true;
        leftIntake.setPower(1);
        rightIntake.setPower(1);
    }

    public void reverseIntake() {
        isIntakeActive = true;
        leftIntake.setPower(-1); // Potência negativa para reverter
        rightIntake.setPower(-1);
    }

    public void stopIntake() {
        isIntakeActive = false;
        leftIntake.setPower(0);
        rightIntake.setPower(0);
    }

    public void wrist(double valueL, double valueR) {
        rightIntakeWrist.setPosition(valueR);
        leftIntakeWrist.setPosition(valueL);
    }

    public void slider(double valueL, double valueR) {
        leftIntakeSlider.setPosition(valueL);
        rightIntakeSlider.setPosition(valueR);
    }
}