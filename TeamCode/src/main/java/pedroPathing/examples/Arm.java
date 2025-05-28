package pedroPathing.examples;

import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
//import com.pedropathing.constants.Constants;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp
@Disabled
public class Arm extends OpMode {

    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);

    private DcMotorEx elevatorMotor;
    private Servo leftArmServo, rightArmServo, clawServo;

    private static final double braçoSubindo  = 0.2;
    private static final double braçoDescendo = 0.8;
    private static final double garraAbrindo  = 0.3;
    private static final double garraFechando = 0.7;

    @Override
    public void init() {

        Constants.setConstants(FConstants.class, LConstants.class);

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);


        elevatorMotor = hardwareMap.get(DcMotorEx.class, "elevatorMotor");
        elevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        leftArmServo  = hardwareMap.get(Servo.class, "leftArmServo");
        rightArmServo = hardwareMap.get(Servo.class, "rightArmServo");
        rightArmServo.setDirection(Servo.Direction.REVERSE);


        clawServo = hardwareMap.get(Servo.class, "clawServo");
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }
    @Override
    public void loop() {

        follower.setTeleOpMovementVectors(
                -gamepad1.left_stick_y,   // forward/back
                -gamepad1.left_stick_x,   // strafe
                -gamepad1.right_stick_x,  // rotate
                false                     // field-centric
        );
        follower.update();

        if (gamepad2.right_trigger > 0.1) {
            elevatorMotor.setPower(1.0);
        } else if (gamepad2.right_bumper) {
            elevatorMotor.setPower(-1.0);
        } else {
            elevatorMotor.setPower(0.0);
        }

        if (gamepad2.y) {
            leftArmServo.setPosition(braçoSubindo);
            rightArmServo.setPosition(braçoSubindo);
        } else {
            leftArmServo.setPosition(braçoDescendo);
            rightArmServo.setPosition(braçoDescendo);
        }

        if (gamepad2.x) {
            clawServo.setPosition(garraFechando);
        } else if (gamepad2.square) {
            clawServo.setPosition(garraAbrindo);
        }

        telemetry.addData("Drive X", follower.getPose().getX());
        telemetry.addData("Drive Y", follower.getPose().getY());
        telemetry.addData("Heading (°)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Elevator ticks", elevatorMotor.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
        elevatorMotor.setPower(0);
    }
}