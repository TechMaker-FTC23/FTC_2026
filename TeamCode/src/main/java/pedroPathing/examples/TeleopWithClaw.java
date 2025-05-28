package pedroPathing.examples;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import com.pedropathing.follower.Follower;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp
public class TeleopWithClaw extends OpMode {

    private Follower follower;

    private Servo clawServo;
    private Servo armServo1;
    private Servo armServo2;
    private Servo wristServo;


    public static final double CLAW_CLOSED_POS = 0.325;
    public static double MAX_OPEN_POS = 0.6;

    private boolean clawOpen = false;
    private boolean lbPreviouslyPressed = false;


    public static final double ARM_UP_POS = 0.0;
    public static final double ARM_DOWN_POS = 0.0;


    public static final double WRIST_UP_POS = 0.0;
    public static final double WRIST_DOWN_POS = 0.0;

    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);

        clawServo = hardwareMap.get(Servo.class, "clawServo");
        armServo1 = hardwareMap.get(Servo.class, "armServo1");
        armServo2 = hardwareMap.get(Servo.class, "armServo2");
        wristServo = hardwareMap.get(Servo.class, "wristServo");

        clawServo.setPosition(CLAW_CLOSED_POS);
        armServo1.setPosition(ARM_DOWN_POS);
        armServo2.setPosition(ARM_DOWN_POS);
        wristServo.setPosition(WRIST_DOWN_POS);
    }

    @Override
    public void loop() {

        if (gamepad1.left_bumper && !lbPreviouslyPressed) {
            clawOpen = !clawOpen;
            clawServo.setPosition(clawOpen ? MAX_OPEN_POS : CLAW_CLOSED_POS);
        }
        lbPreviouslyPressed = gamepad1.left_bumper;

        if (gamepad2.dpad_up) {
            armServo1.setPosition(ARM_UP_POS);
            armServo2.setPosition(ARM_UP_POS);
        }
        else if (gamepad2.dpad_down) {
            armServo1.setPosition(ARM_DOWN_POS);
            armServo2.setPosition(ARM_DOWN_POS);
        }

        if (gamepad2.y) {
            wristServo.setPosition(WRIST_UP_POS);
        } else if (gamepad2.square) {
            wristServo.setPosition(WRIST_DOWN_POS);
        }

        telemetry.addData("Claw", clawOpen ? "ABERTO" : "FECHADO");
        telemetry.addData("Claw Pos", "%.2f", clawServo.getPosition());
        telemetry.addData("Arm1 Pos", "%.2f", armServo1.getPosition());
        telemetry.addData("Arm2 Pos", "%.2f", armServo2.getPosition());
        telemetry.addData("Wrist Pos", "%.2f", wristServo.getPosition());
        telemetry.update();
    }
}
