package pedroPathing.examples;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.pedropathing.follower.Follower;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

import pedroPathing.subsystems.WristSubsystem;
import pedroPathing.subsystems.ClawSubsystem;
import pedroPathing.subsystems.ElevatorSubsystem;

@TeleOp
public class RobotMechanisms extends OpMode {

    private Follower follower;

    // Subsistemas
    private pedroPathing.subsystems.ClawSubsystem claw;
    private pedroPathing.subsystems.WristSubsystem Wrist;
    private pedroPathing.subsystems.ElevatorSubsystem elevator;

    private boolean lbPreviouslyPressed = false;

    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);

        claw = new ClawSubsystem(hardwareMap);
        Wrist = new WristSubsystem(hardwareMap);
        elevator = new ElevatorSubsystem(hardwareMap);

        telemetry.addData("Status", "TeleOp Principal Inicializado");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {

       /* double drive = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;
        follower.setTeleOpMovementVectors(drive, strafe, turn, false);
        follower.update();
       */

        // --- Controle da Garra (Gamepad 1) ---
        if (gamepad1.left_bumper &&!lbPreviouslyPressed) {
            claw.toggleClaw();
        }
        lbPreviouslyPressed = gamepad1.left_bumper;

        // Controle Manual
        if (Math.abs(gamepad2.right_stick_y) > 0.1 &&!elevator.isMovingToPreset()) {
            elevator.setManualPower(-gamepad2.right_stick_y * ElevatorSubsystem.ELEVATOR_MANUAL_SPEED);
        } else if (!elevator.isMovingToPreset()) {
            elevator.setManualPower(0);
        }

        // Presets do Elevador com PID
        if (gamepad2.a) {
            elevator.goToPositionPID(ElevatorSubsystem.ELEVATOR_PRESET_LOW);
        } else if (gamepad2.b) {
            elevator.goToPositionPID(ElevatorSubsystem.ELEVATOR_PRESET_MEDIUM);
        } else if (gamepad2.right_bumper) {
            elevator.goToPositionPID(ElevatorSubsystem.ELEVATOR_PRESET_HIGH);
        } else if (gamepad2.dpad_left) {
            elevator.goToPositionPID(ElevatorSubsystem.ELEVATOR_PRESET_GROUND);
        }



        elevator.update();

        // --- Telemetria ---
        telemetry.addData("Drivetrain Pose", follower.getPose().toString());
        telemetry.addData("Claw", claw.isClawOpen()? "ABERTO" : "FECHADO");
        telemetry.addData("Claw Pos", "%.2f", claw.getClawPosition());
        telemetry.addData("Wrist1 Pos", "%.2f", Wrist.getWrist1Position());
        telemetry.addData("Wrist2 Pos", "%.2f", Wrist.getWrist2Position());
        telemetry.addData("Elevator Target Ticks", elevator.getTargetPosition());
        telemetry.addData("Elevator Current Ticks", elevator.getCurrentPosition());
        telemetry.addData("Elevator MovingToPreset", elevator.isMovingToPreset());
        telemetry.update();
    }
}