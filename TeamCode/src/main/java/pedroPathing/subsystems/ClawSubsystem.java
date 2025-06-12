package pedroPathing.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawSubsystem {
    private Servo clawServo;

    public static final double CLAW_CLOSED_POS = 0.325;
    public static final double CLAW_OPEN_POS = 0.6;

    private boolean clawOpen = false;

    public ClawSubsystem(HardwareMap hardwareMap) {
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        closeClaw();
    }

    public void toggleClaw() {
        if (clawOpen) {
            closeClaw();
        } else {
            openClaw();
        }
    }

    public void openClaw() {
        clawServo.setPosition(CLAW_OPEN_POS);
        clawOpen = true;
    }

    public void closeClaw() {
        clawServo.setPosition(CLAW_CLOSED_POS);
        clawOpen = false;
    }

    public boolean isClawOpen() {
        return clawOpen;
    }

    public double getClawPosition() {
        return clawServo.getPosition();
    }
}