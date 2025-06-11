package pedroPathing.examples;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import com.pedropathing.follower.Follower;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

//Precisa configurar os nomes dos mecanismos no driverHub
@TeleOp
public class TeleopMecanismos extends OpMode {

    private Follower follower;
    private Servo wristServo1;
    private Servo wristServo2;
    private Servo armServo1;
    private Servo armServo2;
    private Servo clawServo;

    private DcMotorEx elevatorMotor;

    private boolean clawOpen = false;
    private boolean lbPreviouslyPressed = false;

    private boolean movingToPreset = false;

    // Claw positions
    private static final double CLAW_CLOSED_POS = 0.325;
    private static final double CLAW_OPEN_POS = 0.6;

    // Arm positions
    private static final double ARM_UP_POS = 0.8;
    private static final double ARM_DOWN_POS = 0.2;

    // Wrist positions
    private static final double WRIST1_UP_POS = 0.0;
    private static final double WRIST1_DOWN_POS = 0.23;

    private static final double WRIST2_UP_POS = 0.23;
    private static final double WRIST2_DOWN_POS = 0.0;

    // Elevator configuration
    private static final int ELEVATOR_MAX_HEIGHT_TICKS = 3000;
    private static final double ELEVATOR_SPEED = 0.3;

    // Elevator presets
    private static final int ELEVATOR_PRESET_LOW = 500;
    private static final int ELEVATOR_PRESET_MEDIUM = 1500;
    private static final int ELEVATOR_PRESET_HIGH = ELEVATOR_MAX_HEIGHT_TICKS;

    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);

        wristServo1 = hardwareMap.get(Servo.class, "wristServo1");
        wristServo2 = hardwareMap.get(Servo.class, "wristServo2");
        armServo1 = hardwareMap.get(Servo.class, "armServo1");
        armServo2 = hardwareMap.get(Servo.class, "armServo2");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        elevatorMotor = hardwareMap.get(DcMotorEx.class, "elevatorMotor");
        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawServo.setPosition(CLAW_CLOSED_POS);
        armServo1.setPosition(ARM_DOWN_POS);
        armServo2.setPosition(ARM_DOWN_POS);

        wristServo1.setPosition(WRIST1_UP_POS);
        wristServo2.setPosition(WRIST2_DOWN_POS);
    }

    @Override
    public void loop() {

        if (gamepad1.left_bumper && !lbPreviouslyPressed) {
            clawOpen = !clawOpen;
            clawServo.setPosition(clawOpen ? CLAW_OPEN_POS : CLAW_CLOSED_POS);
        }
        lbPreviouslyPressed = gamepad1.left_bumper;

        if (gamepad2.dpad_up) {
            armServo1.setPosition(ARM_UP_POS);
            armServo2.setPosition(ARM_UP_POS);
        } else if (gamepad2.dpad_down) {
            armServo1.setPosition(ARM_DOWN_POS);
            armServo2.setPosition(ARM_DOWN_POS);
        }

        if (gamepad2.y) {
            wristServo1.setPosition(WRIST1_UP_POS);
            wristServo2.setPosition(WRIST2_UP_POS);
        } else if (gamepad2.x) {
            wristServo1.setPosition(WRIST1_UP_POS);
            wristServo2.setPosition(WRIST1_UP_POS);
        }

        if (gamepad2.a) {
            moveElevatorToPreset(ELEVATOR_PRESET_LOW);
        } else if (gamepad2.b) {
            moveElevatorToPreset(ELEVATOR_PRESET_MEDIUM);
        } else if (gamepad2.y) {
            moveElevatorToPreset(ELEVATOR_PRESET_HIGH);
        }

        int currentPosition = elevatorMotor.getCurrentPosition();

        if (!movingToPreset) {
            if (gamepad2.right_trigger > 0.4 && currentPosition < ELEVATOR_MAX_HEIGHT_TICKS) {
                elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                elevatorMotor.setPower(ELEVATOR_SPEED);
            } else if (gamepad2.left_trigger > 0.4 && currentPosition > 0) {
                elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                elevatorMotor.setPower(-ELEVATOR_SPEED);
            } else {
                elevatorMotor.setPower(0);
            }
        } else {
            if (!elevatorMotor.isBusy()) {
                elevatorMotor.setPower(0);
                elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                movingToPreset = false;
            }
        }

        telemetry.addData("Claw", clawOpen ? "ABERTO" : "FECHADO");
        telemetry.addData("Claw Pos", "%.2f", clawServo.getPosition());
        telemetry.addData("Arm1 Pos", "%.2f", armServo1.getPosition());
        telemetry.addData("Arm2 Pos", "%.2f", armServo2.getPosition());
        telemetry.addData("Wrist1 Pos", "%.2f", wristServo1.getPosition());
        telemetry.addData("Wrist2 Pos", "%.2f", wristServo2.getPosition());
        telemetry.addData("Elevator Pos", currentPosition);
        telemetry.addData("Moving to Preset", movingToPreset);
        telemetry.update();
    }

    private void moveElevatorToPreset(int targetPosition) {
        elevatorMotor.setTargetPosition(targetPosition);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorMotor.setPower(ELEVATOR_SPEED);
        movingToPreset = true;
    }
}
