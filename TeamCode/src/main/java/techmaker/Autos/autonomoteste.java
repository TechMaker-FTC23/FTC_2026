package techmaker.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import techmaker.constants.Constants;
import techmaker.constants.FConstants;
import techmaker.constants.LConstants;
import techmaker.subsystems.ClawSubsystem;
import techmaker.subsystems.ElevatorSubsystem;
import techmaker.subsystems.IntakeSubsystem;
import techmaker.util.StateMachine;

@Autonomous
public class autonomoteste extends LinearOpMode {
    private ClawSubsystem claw;
    private ElevatorSubsystem elevator;
    private IntakeSubsystem intake;
    private Telemetry telemetryA;
    private Follower follower;
    private Limelight3A limelight3A;
    private final Pose startPose = new Pose(6.150190698818898, 62.440310500738185, Math.toRadians(180));
    private final Pose BargeUp = new Pose(32.40432859405758, -3.973167599655512, Math.toRadians(190.19982702485984));
    private final Pose BargeMiddle = new Pose(-8.774839386226624, 53.486424243356296, Math.toRadians(257.56982137500916));
    private final Pose SpikeMarkC = new Pose(52.83622201033465, 54.25664946788878, Math.toRadians(268.48007235794887));
    private final Pose SpikeMarkD = new Pose(44.25882174274115, 54.10334068959153, Math.toRadians(266.017482969839));
    private final Pose SpikeMarkE = new Pose(50.39361908679872, 50.80521711214321, Math.toRadians(295.18674039010233));
    private final Pose basketPose = new Pose(52, 59, Math.toRadians(227));
    @Override
    public void runOpMode() throws InterruptedException {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setPose(startPose);
        follower.update();
        claw = new ClawSubsystem(hardwareMap);
        elevator = new ElevatorSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap, false);

        claw.setState(ClawSubsystem.ClawState.TRAVEL);
        claw.setClawOpen(false);
        claw.setArmPosition(ClawSubsystem.ARM_LEFT_TRAVEL_CLAW, ClawSubsystem.ARM_RIGHT_TRAVEL_CLAW);

        intake.wrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MIN, IntakeSubsystem.RIGHT_INTAKE_WRIST_MIN);
        intake.sliderMin();
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(0);
        limelight3A.start();
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        updatePoseLimelight();
        telemetryA.addData("Status", "Autônomo Inicializado.");
        telemetryA.addData("Start",follower.getPose());
        telemetryA.update();

        waitForStart();

        if (opModeIsActive() &&!isStopRequested()) {
            intake.sliderMin();
            claw.setClawOpen(false);

            telemetryA.addData("CICLO 1", "Indo para a Basket com o pré-carregado");
            telemetryA.update();
            executePathToPose(basketPose, true, null);

            elevator.goToPositionPID(ElevatorSubsystem.ELEVATOR_PRESET_HIGH);

            while (!elevator.atTargetPosition(20)) {
                claw.setClawOpen(false); // MANTÉM A GARRA FECHADA COM FIRMEZA
                elevator.update(telemetry);
            }

            claw.setWristPosition(ClawSubsystem.WRIST_LEFT_SCORE_CLAW,ClawSubsystem.WRIST_RIGHT_SCORE_CLAW);
            claw.setArmPosition(ClawSubsystem.ARM_LEFT_SCORE_CLAW, ClawSubsystem.ARM_RIGHT_SCORE_CLAW);
            sleep(1500);

            claw.setClawOpen(true); // Agora sim, abre para pontuar
            sleep(1500);

            claw.setArmPosition(ClawSubsystem.ARM_LEFT_TRAVEL_CLAW, ClawSubsystem.ARM_RIGHT_TRAVEL_CLAW);
            elevator.goToPositionPID(ElevatorSubsystem.ELEVATOR_PRESET_GROUND);
            sleep(600);

            //... O resto do seu código continua aqui...
            telemetryA.addData("CICLO 2", "Indo para o SpikeMark C");
            telemetryA.update();
            executePathToPose(SpikeMarkC, false, null);

            //... etc...
        }
    }

    private void executePathToPose(Pose targetPose, boolean isCurve, Pose controlPoint) {
        if (!opModeIsActive()) return;

        Pose startOfPathPose = follower.getPose();
        PathChain path;

        if (isCurve && controlPoint!= null) {
            path = follower.pathBuilder()
                    .addPath(new BezierCurve(new Point(startOfPathPose), new Point(controlPoint), new Point(targetPose)))
                    .setLinearHeadingInterpolation(startOfPathPose.getHeading(), targetPose.getHeading())
                    .build();
        } else {
            path = follower.pathBuilder()
                    .addPath(new BezierLine(new Point(startOfPathPose), new Point(targetPose)))
                    .setLinearHeadingInterpolation(startOfPathPose.getHeading(), targetPose.getHeading())
                    .build();
        }

        follower.followPath(path, true);
        waitForPathToFinish();
    }

    private void waitForPathToFinish() {
        while (opModeIsActive() &&!isStopRequested() && follower.isBusy()) {
            follower.update();
            updatePoseLimelight(); // Atualiza a pose continuamente durante o caminho
            telemetryA.addData("Pose", follower.getPose());
            telemetryA.update();
        }
    }

    void updatePoseLimelight(){
        double currentYawDegrees = Math.toDegrees(follower.getPose().getHeading());
        limelight3A.updateRobotOrientation(currentYawDegrees);

        LLResult result = limelight3A.getLatestResult();

        if (result!= null && result.isValid()) {
            Pose3D botpose = result.getBotpose_MT2();
            if (botpose!= null) {
                // Converte a Pose3D da Limelight (metros) para a Pose 2D do Pedro Pathing (polegadas)
                Pose visionPose = new Pose(
                        botpose.getPosition().x * 39.37, // metros para polegadas
                        botpose.getPosition().y * 39.37, // metros para polegadas
                        botpose.getOrientation().getYaw(AngleUnit.RADIANS)
                );
                follower.setPose(visionPose);
            }
        }
    }
}