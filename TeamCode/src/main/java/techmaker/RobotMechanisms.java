package techmaker;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.pedropathing.follower.Follower;

import techmaker.constants.FConstants;
import techmaker.constants.LConstants;

import techmaker.subsystems.ClawSubsystem;
import techmaker.subsystems.ElevatorSubsystem;
import techmaker.subsystems.IntakeSubsystem;

@TeleOp(name = "Controle Principal do Robô", group = "PedroPathing")
public class RobotMechanisms extends OpMode {

    //private Follower follower;
    private ClawSubsystem claw;
    //private ElevatorSubsystem elevator;
    private IntakeSubsystem intake;
    private Follower follower;
    private final Pose startPose = new Pose(0,0,0);
    @Override
    public void init() {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        // --- Inicialização dos Subsistemas ---
        claw = new ClawSubsystem(hardwareMap);
        //elevator = new ElevatorSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap, false);

        telemetry.addData("Status", "TeleOp Principal Inicializado");
        telemetry.addData("Dashboard", "Conecte-se em 192.168.43.1:8080");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();

    }

    @Override
    public void loop() {
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        follower.update();

        if (gamepad1.dpad_right) {
            intake.wrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MAX,IntakeSubsystem.RIGHT_INTAKE_WRIST_MAX);
            intake.startIntake();
        }
        if (gamepad1.dpad_left) {
            intake.wrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MIN, IntakeSubsystem.RIGHT_INTAKE_WRIST_MIN);
            intake.stopIntake();
        }
        if (gamepad1.circle) {
            intake.slider(IntakeSubsystem.LEFT_INTAKE_SLIDER_MAX,IntakeSubsystem.RIGHT_INTAKE_SLIDER_MAX);
        } else {
            intake.slider(IntakeSubsystem.LEFT_INTAKE_SLIDER_MIN,IntakeSubsystem.RIGHT_INTAKE_SLIDER_MIN);
        }

        if(gamepad1.dpad_down){
            claw.clawArm(ClawSubsystem.medArml,ClawSubsystem.medArmR);
        }
        else{
            claw.clawArm(ClawSubsystem.minArmL,ClawSubsystem.minArmR);
        }
        if(gamepad1.dpad_up){
            claw.clawArm(ClawSubsystem.maxArmL,ClawSubsystem.maxArmR);
        }
        else{
            claw.clawArm(ClawSubsystem.minArmL,ClawSubsystem.minArmR);
        }
        if(gamepad1.square){
            claw.clawWrist(ClawSubsystem.maxWristL,ClawSubsystem.maxWristR);
        }
        else{
            claw.clawWrist(ClawSubsystem.minWristL,ClawSubsystem.minWristR);
        }
        if(gamepad1.triangle){
            claw.middleClaw(ClawSubsystem.maxClaw);
        }
        else{
            claw.middleClaw(ClawSubsystem.minClaw);
        }

        claw.update(telemetry);
        //elevator.update(telemetry);
        intake.update(telemetry);
        telemetry.update();
    }
}