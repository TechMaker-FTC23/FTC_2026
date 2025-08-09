package techmaker.subsystems;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import techmaker.constants.Constants;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.concurrent.TimeUnit;

@Config
public class IntakeSubsystem {

    public static final String COLOR_SENSOR_NAME = "colorSensor";

    public static double LEFT_INTAKE_WRIST_MAX = 0.9;
    public static double RIGHT_INTAKE_WRIST_MAX = 0.1;
    public static double LEFT_INTAKE_WRIST_MIN = 0.32;
    public static double RIGHT_INTAKE_WRIST_MIN = 0.68;

    public static double LEFT_INTAKE_SLIDER_MAX = 0.4;
    public static double RIGHT_INTAKE_SLIDER_MAX = 0.6;
    public static double LEFT_INTAKE_SLIDER_MED = 0.7;
    public static double RIGHT_INTAKE_SLIDER_MED = 0.3;
    public static double LEFT_INTAKE_SLIDER_MIN = 0.8;
    public static double RIGHT_INTAKE_SLIDER_MIN = 0.2;

    // --- Constantes para Detecção de Cor no Espaço HSV ---
    public static double YELLOW_HUE_MIN = 40.0;
    public static double YELLOW_HUE_MAX = 70.0;
    public static double RED_HUE_MIN_1 = 0.0;
    public static double RED_HUE_MAX_1 = 20.0;
    public static double RED_HUE_MIN_2 = 340.0;
    public static double RED_HUE_MAX_2 = 360.0;
    public static double BLUE_HUE_MIN = 210.0;
    public static double BLUE_HUE_MAX = 260.0;
    public static double MIN_SATURATION = 0.5;
    public static double MIN_VALUE = 0.2;

    public enum CaptureState { IDLE, SEARCHING, CAPTURED }

    private final CRServo leftIntake;
    private final CRServo rightIntake;
    private final CRServo middleIntake;
    private final Servo leftIntakeWrist;
    private final Servo rightIntakeWrist;
    private final Servo leftIntakeSlider;
    private final Servo rightIntakeSlider;
    private NormalizedColorSensor colorSensor;

    private final ElapsedTime timer = new ElapsedTime();
    private double lastLeftSliderPos;
    private double lastRightSliderPos;
    private final boolean isRedAlliance;
    private final float[] hsvValues = new float[3];
    private CaptureState captureState = CaptureState.IDLE;

    public IntakeSubsystem(HardwareMap hardwareMap, boolean isRedAlliance) {
        leftIntake = hardwareMap.get(CRServo.class,Constants.Intake.Left);
        rightIntake = hardwareMap.get(CRServo.class, Constants.Intake.Right);
        middleIntake = hardwareMap.get(CRServo.class, Constants.Intake.Middle);
        leftIntakeWrist = hardwareMap.get(Servo.class, Constants.Intake.LeftWrist);
        rightIntakeWrist = hardwareMap.get(Servo.class, Constants.Intake.RightWrist);
        leftIntakeSlider = hardwareMap.get(Servo.class, Constants.Intake.LeftSlider);
        rightIntakeSlider = hardwareMap.get(Servo.class, Constants.Intake.RightSlider);
        try {
            colorSensor = hardwareMap.get(NormalizedColorSensor.class, COLOR_SENSOR_NAME);
        } catch (Exception e) {
            colorSensor = null;
        }
        this.isRedAlliance = isRedAlliance;
        leftIntake.setDirection(CRServo.Direction.FORWARD);
        rightIntake.setDirection(CRServo.Direction.REVERSE);
        middleIntake.setDirection(CRServo.Direction.FORWARD);
        lastLeftSliderPos = LEFT_INTAKE_SLIDER_MIN;
        lastRightSliderPos = RIGHT_INTAKE_SLIDER_MIN;
        slider(lastLeftSliderPos, lastRightSliderPos);
        timer.reset();
        stopIntake();
    }

    public boolean isPixelDetected() {
        if (colorSensor == null) return false;
        /*NormalizedRGBA colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);

        float hue = hsvValues[0], saturation = hsvValues[1], value = hsvValues[2];

        if (saturation < MIN_SATURATION || value < MIN_VALUE) return false;

        if (hue >= YELLOW_HUE_MIN && hue <= YELLOW_HUE_MAX) return true;

        if (isRedAlliance) {
            return (hue >= RED_HUE_MIN_1 && hue <= RED_HUE_MAX_1) || (hue >= RED_HUE_MIN_2 && hue <= RED_HUE_MAX_2);
        } else {
            return (hue >= BLUE_HUE_MIN && hue <= BLUE_HUE_MAX);
        }*/
        return (((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM)<3);
    }

    public void startAutomaticCapture() {
        if (captureState == CaptureState.IDLE) captureState = CaptureState.SEARCHING;
    }

    public void updateAutomaticCapture() {
        if (captureState == CaptureState.SEARCHING) {
            startIntake();
            if (isPixelDetected()) captureState = CaptureState.CAPTURED;
        } else if (captureState == CaptureState.CAPTURED) {
            stopIntake();
        }
    }

    public boolean isCaptureComplete() {
        return captureState == CaptureState.CAPTURED;
    }

    public void resetCaptureState() {
        captureState = CaptureState.IDLE;
    }

    public void startIntake() {
        leftIntake.setPower(-1.0);
        rightIntake.setPower(-1.0);
        middleIntake.setPower(-1.0);
    }

    public void reverseIntake() {
        leftIntake.setPower(1.0);
        rightIntake.setPower(1.0);
    }

    public void stopIntake() {
        leftIntake.setPower(0);
        rightIntake.setPower(0);
        middleIntake.setPower(0);
    }

    public void wrist(double leftPosition, double rightPosition) {
        leftIntakeWrist.setPosition(leftPosition);
        rightIntakeWrist.setPosition(rightPosition);
    }

    private void slider(double leftPosition, double rightPosition) {
        leftIntakeSlider.setPosition(leftPosition);
        rightIntakeSlider.setPosition(rightPosition);
        lastLeftSliderPos = leftPosition;
        lastRightSliderPos = rightPosition;
    }

    // --- NOVOS MÉTODOS DE CONVENIÊNCIA PARA O SLIDER ---
    public void sliderMax() {
        slider(LEFT_INTAKE_SLIDER_MAX, RIGHT_INTAKE_SLIDER_MAX);
    }

    public void sliderMedium() {
        slider(LEFT_INTAKE_SLIDER_MED, RIGHT_INTAKE_SLIDER_MED);
    }

    public void sliderMin() {
        slider(LEFT_INTAKE_SLIDER_MIN, RIGHT_INTAKE_SLIDER_MIN);
    }

    public void maintainSliderPosition() {
        leftIntakeSlider.setPosition(lastLeftSliderPos);
        rightIntakeSlider.setPosition(lastRightSliderPos);
    }
    public void wristMin(){
        wrist(LEFT_INTAKE_WRIST_MIN, RIGHT_INTAKE_WRIST_MIN);
    }

    public void update(Telemetry telemetry) {
        if (timer.time(TimeUnit.MILLISECONDS) > 50) {
            timer.reset();
            telemetry.addData("Aliança", isRedAlliance ? "Vermelha" : "Azul");
            telemetry.addData("Estado da Captura", captureState);
            if (colorSensor != null) {
                NormalizedRGBA colors = colorSensor.getNormalizedColors();
                Color.colorToHSV(colors.toColor(), hsvValues);
                telemetry.addData("Pixel Válido Detectado", isPixelDetected() ? "SIM" : "NÃO");
                telemetry.addData("--- Sensor HSV ---", "");
                telemetry.addData("Hue (Matiz)", "%.1f", hsvValues[0]);
                telemetry.addData("Saturation (Saturação)", "%.3f", hsvValues[1]);
                telemetry.addData("Value (Valor/Brilho)", "%.3f", hsvValues[2]);
            } else {
                telemetry.addData("SENSOR DE COR", "NÃO CONECTADO");
            }
        }
    }
}
