package techmaker;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

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

    @Override
    public void init() {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        //follower = new Follower(hardwareMap, FConstants.class, LConstants.class);

        // --- Inicialização dos Subsistemas ---
        claw = new ClawSubsystem(hardwareMap); // ALTERADO: Usa a nova variável
        //elevator = new ElevatorSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap, false);

        telemetry.addData("Status", "TeleOp Principal Inicializado");
        telemetry.addData("Dashboard", "Conecte-se em 192.168.43.1:8080");
        telemetry.update();
    }

    @Override
    public void start() {
        //follower.startTeleopDrive();

    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up) {
            intake.wrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MAX,IntakeSubsystem.RIGHT_INTAKE_WRIST_MAX);
        } else {
            intake.wrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MIN,IntakeSubsystem.RIGHT_INTAKE_WRIST_MIN);
        }
        if (gamepad1.dpad_left) {
            intake.startIntake();
        } else {
            intake.stopIntake();
        }

        if (gamepad1.circle) {
            intake.slider(IntakeSubsystem.LEFT_INTAKE_SLIDER_MAX,IntakeSubsystem.RIGHT_INTAKE_SLIDER_MAX);
        } else {
            intake.slider(IntakeSubsystem.LEFT_INTAKE_SLIDER_MIN,IntakeSubsystem.RIGHT_INTAKE_SLIDER_MIN);
        }

        if(gamepad1.dpad_down){
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