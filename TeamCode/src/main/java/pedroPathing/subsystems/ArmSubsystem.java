package pedroPathing.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmSubsystem {
    private Servo wristServo1;
    private Servo wristServo2;
    private Servo armServo1;
    private Servo armServo2;

    // Posições do Braço
    public static final double ARM_UP_POS = 0.8;
    public static final double ARM_DOWN_POS = 0.2;

    // Posições do Pulso
    public static final double WRIST1_UP_POS = 0.0;
    public static final double WRIST1_DOWN_POS = 0.23;
    public static final double WRIST2_UP_POS = 0.23; // No seu código original, gamepad2.y e gamepad2.x tinham lógicas diferentes para wrist2
    public static final double WRIST2_DOWN_POS = 0.0;

    public ArmSubsystem(HardwareMap hardwareMap) {
        wristServo1 = hardwareMap.get(Servo.class, "wristServo1");
        wristServo2 = hardwareMap.get(Servo.class, "wristServo2");
        armServo1 = hardwareMap.get(Servo.class, "armServo1");
        armServo2 = hardwareMap.get(Servo.class, "armServo2");

        // Definir posições iniciais
        setArmPositionDown();
        setWrist1PositionUp(); // Ou a posição inicial que você preferir
        setWrist2PositionDown(); // Ou a posição inicial que você preferir
    }

    public void setArmPositionUp() {
        armServo1.setPosition(ARM_UP_POS);
        armServo2.setPosition(ARM_UP_POS);
    }

    public void setArmPositionDown() {
        armServo1.setPosition(ARM_DOWN_POS);
        armServo2.setPosition(ARM_DOWN_POS);
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