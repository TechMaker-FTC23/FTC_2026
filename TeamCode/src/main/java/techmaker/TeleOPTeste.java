package techmaker;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import techmaker.constants.Constants;
import techmaker.constants.FConstants;
import techmaker.constants.LConstants;
import techmaker.subsystems.ClawSubsystem;
import techmaker.subsystems.ElevatorSubsystem;
import techmaker.subsystems.IntakeSubsystem;
import techmaker.util.DataStorage;
import techmaker.util.StateMachine;

@TeleOp(name = "Teste de mecanismos")
public class TeleOPTeste extends OpMode {
    private ClawSubsystem claw;
    private ElevatorSubsystem elevator;
    private IntakeSubsystem intake;
    private Follower follower;
    private Limelight3A limelight;
    private StateMachine state = StateMachine.IDLE;
    private StateMachine stateClawSample = StateMachine.CLAW_SPECIMENT;
    private final Pose startPose = new Pose(0, 0, Math.PI);
    private final Pose BargeUp = new Pose(95 / 2.54, 80 / 2.54, Math.toRadians(0));
    private final Pose BargeMiddle = new Pose(100 / 2.54, 30 / 2.54, Math.toRadians(-90));
    private final Pose Basket = new Pose(51.65671040692668, 58.230110228531004, Math.toRadians(221.45235477987202));
    private final ElapsedTime timer = new ElapsedTime();
    private long timeout = 0;


    @Override
    public void init() {
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(DataStorage.robotPose);
        follower.update();
        claw = new ClawSubsystem(hardwareMap);
        elevator = new ElevatorSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap, false);
        intake.setIsRedAlliance(DataStorage.allianceColor == Constants.Intake.SampleColor.Red);


        // MUDANÇA: Define a posição inicial segura da garra com um único comando.
        claw.setState(ClawSubsystem.ClawState.TRAVEL);
        claw.setClawOpen(true); // Começa com a garra aberta
        claw.setArmPosition(ClawSubsystem.ARM_LEFT_TRAVEL_CLAW, ClawSubsystem.ARM_RIGHT_TRAVEL_CLAW);
        intake.wrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MIN, IntakeSubsystem.RIGHT_INTAKE_WRIST_MIN);
        intake.sliderMin();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        telemetry.addData("Status", "TeleOp Principal Inicializado");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        intake.sliderMin();
        intake.update(telemetry);
        telemetry.update();
        follower.setStartingPose(DataStorage.robotPose);
        follower.update();
        updatePoseFromLimelight();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        timer.reset();
    }


    @Override
    public void loop() {
       follower.update();
       if(gamepad1.triangle){
           claw.setState(ClawSubsystem.ClawState.INTAKE);
       }
        if(gamepad1.circle){
            claw.setState(ClawSubsystem.ClawState.TRAVEL);
        }
        if(gamepad1.cross){
            claw.setState(ClawSubsystem.ClawState.SCORE);
        }
        if(gamepad1.dpad_up){
            tranferSample();
        }
        if(gamepad1.dpad_down){
            claw.setClawOpen(true);
        }


        intake.sliderMin();
        intake.wristMin();
        Pose pose = follower.getPose();
        //claw.update(telemetry);
        elevator.update(telemetry);
        intake.update(telemetry);
        telemetry.addData("Main State", state);
        //telemetry.addData("Claw State", stateClawSample);
        telemetry.addData("Pose", pose);
        telemetry.update();

    }
    public void tranferSample(){

        intake.reverseIntake();
        double time = getRuntime()+0.1;
        while(getRuntime()<time){

        }
        intake.stopIntake();
        claw.setClawOpen(false);


    }
    private void updatePoseFromLimelight() {
        double currentYawDegrees = Math.toDegrees(follower.getPose().getHeading());
        limelight.updateRobotOrientation(currentYawDegrees);

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return;
        }

        Pose3D botpose = result.getBotpose_MT2();
        if (botpose != null) {
            Pose visionPose = new Pose(
                    botpose.getPosition().x * 39.37, // metros para polegadas
                    botpose.getPosition().y * 39.37, // metros para polegadas
                    botpose.getOrientation().getYaw(AngleUnit.RADIANS)
            );
            follower.setPose(visionPose);
            follower.update();
        }
    }
    public Pose invertPose(Pose pose){
        return new Pose(-pose.getX(),-pose.getY(),pose.getHeading()-Math.PI);
    }

}
