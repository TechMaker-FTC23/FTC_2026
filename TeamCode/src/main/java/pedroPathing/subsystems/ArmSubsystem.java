package pedroPathing.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmSubsystem {
    private Servo wristServo1;
    private Servo wristServo2;
    private Servo armServo1;
    private Servo armServo2;

    // Posições do Braço
    public static final double ARM1_UP_POS = 0.8;
    public static final double ARM1_DOWN_POS = 0.2;
    public static final double ARM2_UP_POS = 0.2;
    public static final double ARM2_DOWN_POS = 0.8;

    public static final double WRIST1_UP_POS = 0.0;
    public static final double WRIST1_DOWN_POS = 0.23;
    public static final double WRIST2_UP_POS = 0.23;
    public static final double WRIST2_DOWN_POS = 0.0;
    public static final double WRIST1_MEDIUM_POS = 0.115;
    public static final double WRIST2_MEDIUM_POS = 0.115;

    public ArmSubsystem(HardwareMap hardwareMap) {
        wristServo1 = hardwareMap.get(Servo.class, "wristServo1");
        wristServo2 = hardwareMap.get(Servo.class, "wristServo2");
        armServo1 = hardwareMap.get(Servo.class, "armServo1");
        armServo2 = hardwareMap.get(Servo.class, "armServo2");


        setArmPositionDown();
        setWrist1PositionUp();
        setWrist2PositionDown();
    }

    public void setArmPositionUp() {
        armServo1.setPosition(ARM1_UP_POS);
        armServo2.setPosition(ARM2_UP_POS);
    }

    public void setArmPositionDown() {
        armServo1.setPosition(ARM1_DOWN_POS);
        armServo2.setPosition(ARM2_DOWN_POS);
    }

    public void setWrist1PositionUp() {
        wristServo1.setPosition(WRIST1_UP_POS);
    }

    public void setWrist1PositionDown() {
        wristServo1.setPosition(WRIST1_DOWN_POS);
    }

    public void setWrist2PositionUp() {
        wristServo2.setPosition(WRIST2_UP_POS);
    }

    public void setWrist2PositionDown() {
        wristServo2.setPosition(WRIST2_DOWN_POS);
    }

    public void setWrist1PositionMedium() {
        wristServo2.setPosition(WRIST1_MEDIUM_POS);
    }

    public void setWrist2PositionMedium() {
        wristServo2.setPosition(WRIST2_MEDIUM_POS);
    }


    // Método para a combinação do gamepad2.x original
    public void setBothWristsCustom(double wrist1Pos, double wrist2Pos) {
        wristServo1.setPosition(wrist1Pos);
        wristServo2.setPosition(wrist2Pos);
    }


    public double getArm1Position() {
        return armServo1.getPosition();
    }

    public double getArm2Position() {
        return armServo2.getPosition();
    }

    public double getWrist1Position() {
        return wristServo1.getPosition();
    }

    public double getWrist2Position() {
        return wristServo2.getPosition();
    }
}