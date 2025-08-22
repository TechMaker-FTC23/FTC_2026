package techmaker.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import techmaker.constants.FConstants;
import techmaker.constants.LConstants;
import techmaker.subsystems.ClawSubsystem;
import techmaker.subsystems.ElevatorSubsystem;
import techmaker.subsystems.IntakeSubsystem;
import techmaker.subsystems.LimelightSubsystem;

@Autonomous
public class AZUL_AutoBasket extends LinearOpMode {
    private ClawSubsystem claw;
    private ElevatorSubsystem elevator;
    private IntakeSubsystem intake;
    private Telemetry telemetryA;
    private Follower follower;
    private LimelightSubsystem limelight3A;
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
        limelight3A = new LimelightSubsystem(hardwareMap);
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        while(!isStarted()) {
            Pose actual = follower.getPose();
            follower.setPose(limelight3A.updatePoseLimelight(actual));
            follower.update();
            telemetryA.addData("Status", "Autônomo Inicializado.");
            if (limelight3A.isAprilTagVisible()) {
                actual = follower.getPose();
            } else {
                actual = new Pose(0, 0, 0);
            }
            telemetryA.addData("Start", actual);

            telemetryA.addData("Pose inicial correta", actual.roughlyEquals(startPose, 1) && limelight3A.isAprilTagVisible());
            telemetryA.addData("Cor", intake.getColor());
            telemetryA.update();
            idle();
        }
        while (opModeIsActive() && !isStopRequested()) {
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

            claw.setWristPosition(ClawSubsystem.WRIST_LEFT_SCORE_CLAW, ClawSubsystem.WRIST_RIGHT_SCORE_CLAW);
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

        Pose startOfPathPose = follower.getPose();
        PathChain path;

        if (isCurve && controlPoint != null) {
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
        while (follower.isBusy() && opModeIsActive() && !isStopRequested()) {
            follower.update();
            follower.setPose(limelight3A.updatePoseLimelight(follower.getPose()));
            telemetryA.addData("Pose", follower.getPose());
            telemetryA.update();
        }
    }


}