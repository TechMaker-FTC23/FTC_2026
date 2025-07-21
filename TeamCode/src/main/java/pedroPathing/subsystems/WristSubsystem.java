package pedroPathing.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.config.Config;

@Config
public class WristSubsystem {
    private Servo wristServo1;
    private Servo wristServo2;

    public static double WRIST1_UP_POS = 0.3;
    public static double WRIST1_DOWN_POS = 0.3;
    public static double WRIST1_MEDIUM_POS = 0.115;

    public static double WRIST2_UP_POS = 0.3;
    public static double WRIST2_DOWN_POS = 0.0;
    public static double WRIST2_MEDIUM_POS = 0.115;

    private double wrist1TargetPos;
    private double wrist2TargetPos;

    private boolean wristAberto = false; // começa fechado

    public WristSubsystem(HardwareMap hardwareMap) {
        wristServo1 = hardwareMap.get(Servo.class, "wristServo1");
        wristServo2 = hardwareMap.get(Servo.class, "wristServo2");

        closeWrist(); // começa fechado
    }

    public void openWrist() {
        wristAberto = true;
        setWristTarget(WRIST1_UP_POS, WRIST2_UP_POS);
    }

    public void closeWrist() {
        wristAberto = false;
        setWristTarget(WRIST1_DOWN_POS, WRIST2_DOWN_POS);
    }

    public void setWristMedium() {
        setWristTarget(WRIST1_MEDIUM_POS, WRIST2_MEDIUM_POS);
    }

    public boolean isWristAberto() {
        return wristAberto;
    }

    public void setWristTarget(double wrist1Pos, double wrist2Pos) {
        wrist1TargetPos = wrist1Pos;
        wrist2TargetPos = wrist2Pos;

        wristServo1.setPosition(1.0 - wrist1TargetPos);
        wristServo2.setPosition(1.0 - wrist2TargetPos); // inversão para servo espelhado
    }

    public void updateWrist() {
        wristServo1.setPosition(1.0 - wrist1TargetPos);
        wristServo2.setPosition(1.0 - wrist2TargetPos);
    }

    public double getWrist1Position() {
        return wristServo1.getPosition();
    }

    public double getWrist2Position() {
        return wristServo2.getPosition();
    }
}
