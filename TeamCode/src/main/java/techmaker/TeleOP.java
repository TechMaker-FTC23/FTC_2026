package techmaker;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import techmaker.constants.FConstants;
import techmaker.constants.LConstants;
import techmaker.subsystems.ClawSubsystem;
import techmaker.subsystems.ElevatorSubsystem;
import techmaker.subsystems.IntakeSubsystem;
import techmaker.util.StateMachine;

@TeleOp(name = "TeleOp Posições Fixas")
public class TeleOP extends OpMode {
    private ClawSubsystem claw;
    private ElevatorSubsystem elevator;
    private IntakeSubsystem intake;
    private Follower follower;
    private Limelight3A limelight;
    private StateMachine state = StateMachine.IDLE;
    private StateMachine stateClawSample = StateMachine.CLAW_SPECIMENT;
    private final Pose startPose = new Pose(0,0,180);
    private final Pose BargeUp = new Pose(32.40432859405758, -3.973167599655512, Math.toRadians(190.19982702485984));
    private final Pose BargeMiddle = new Pose(-8.774839386226624, 53.486424243356296, Math.toRadians(257.56982137500916));
    private final Pose Basket = new Pose(51.65671040692668, 58.230110228531004, Math.toRadians(221.45235477987202));
    private final ElapsedTime timer = new ElapsedTime();
    private long timeout = 0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        claw = new ClawSubsystem(hardwareMap);
        elevator = new ElevatorSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap, false);

        claw.setState(ClawSubsystem.ClawState.TRAVEL);
        claw.setClawOpen(true);
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
    }
    @Override
    public void start() {
        follower.startTeleopDrive();
        timer.reset();
    }


    @Override
    public void loop() {
        follower.update();

        if (follower.isBusy()) {
        } else {
            Pose visionPose = getVisionPose();
            if (visionPose!= null) {
                follower.setPose(visionPose);
            }

            // 2. Lê os joysticks para o controle do piloto.
            double y_stick = -gamepad1.left_stick_y;
            double x_stick = gamepad1.left_stick_x;
            double turn_stick = gamepad1.right_stick_x;
            double heading = follower.getPose().getHeading();
            double rotatedX = x_stick * Math.cos(heading) + y_stick * Math.sin(heading);
            double rotatedY = -x_stick * Math.sin(heading) + y_stick * Math.cos(heading);
            follower.setTeleOpMovementVectors(rotatedY, rotatedX, turn_stick, true);

            // 3. Verifica se o piloto quer iniciar um NOVO caminho automático.
            if (gamepad1.cross){
                PathChain BASKET = follower.pathBuilder()
                        .addPath(new BezierLine(new Point(follower.getPose()), new Point(Basket)))
                        .build();
                follower.followPath(BASKET, true);

            } else if (gamepad1.square){
                PathChain bargeMiddle = follower.pathBuilder()
                        .addPath(new BezierLine(new Point(follower.getPose()), new Point(BargeMiddle)))
                        .build();
                follower.followPath(bargeMiddle, true);

            } else if (gamepad1.circle){
                PathChain bargeUp = follower.pathBuilder()
                        .addPath(new BezierLine(new Point(follower.getPose()), new Point(BargeUp)))
                        .build();
                follower.followPath(bargeUp, true);
            }
        }

        intake.maintainSliderPosition();

        if (gamepad2.triangle && state == StateMachine.IDLE) {
            state = StateMachine.START_INTAKE;
            intake.wrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MAX, IntakeSubsystem.RIGHT_INTAKE_WRIST_MAX);
            intake.startIntake();
            timeout = 200;
            timer.reset();
        } else if (state == StateMachine.INTAKING &&(gamepad2.circle

                | intake.isPixelDetected())){
            state = StateMachine.INTAKE_DETECTING;
            timeout = 100;
            timer.reset();
        }

        if (gamepad2.right_bumper && stateClawSample == StateMachine.CLAW_SPECIMENT) {
            stateClawSample = StateMachine.CLAW_SAMPLE;
            claw.setArmPosition(ClawSubsystem.ARM_LEFT_INTAKE_CLAW, ClawSubsystem.ARM_RIGHT_INTAKE_CLAW);
            claw.setClawOpen(false);
            timeout = 100;
            timer.reset();
        } else if (gamepad2.left_bumper && stateClawSample == StateMachine.DELIVERY_SPECIMENT) {
            stateClawSample = StateMachine.CLAW_RETRACT;
            claw.setClawOpen(true);
            timeout = 200;
            timer.reset();
        }

        if (gamepad2.cross && state == StateMachine.IDLE) {
            state = StateMachine.AUTO_CYCLE_START;
            intake.sliderMax();
            timeout = 100;
            timer.reset();
        }

        if (gamepad2.dpad_right){
            intake.reverseIntake();
        }

        if(state == StateMachine.REVERTING_INTAKE &&!intake.isPixelDetected()){
            state = StateMachine.RETURNING_INTAKE;
            intake.wrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MIN, IntakeSubsystem.RIGHT_INTAKE_WRIST_MIN);
            intake.stopIntake();
            timeout = 100;
            timer.reset();
        }
        if (timer.milliseconds() > timeout) {
            if (state == StateMachine.START_INTAKE) {
                intake.sliderMax();
                state = StateMachine.INTAKING;
            }
            else if(state==StateMachine.INTAKE_DETECTING){
                if(intake.isSampleCorrectAlliance()){
                    gamepad1.rumble(200);
                    state = StateMachine.RETURNING_INTAKE;
                    intake.wrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MIN, IntakeSubsystem.RIGHT_INTAKE_WRIST_MIN);
                    intake.stopIntake();
                    timeout = 30;
                    timer.reset();
                }
                else{
                    intake.reverseIntake();
                    state = StateMachine.REVERTING_INTAKE;
                }
            }
            else if (state == StateMachine.RETURNING_INTAKE) {
                intake.stopIntake();
                intake.sliderMin();
                state = StateMachine.IDLE;
            } else if (state == StateMachine.AUTO_CYCLE_START) {
                intake.wrist(IntakeSubsystem.LEFT_INTAKE_WRIST_MAX, IntakeSubsystem.RIGHT_INTAKE_WRIST_MAX);
                intake.startIntake();
                state = StateMachine.AUTO_INTAKING;
                timeout = 400;
                timer.reset();
            } else if (state == StateMachine.AUTO_INTAKING) {
                claw.setClawOpen(false);
                claw.setState(ClawSubsystem.ClawState.INTAKE);
                intake.reverseIntake();
                state = StateMachine.AUTO_GRAB;
                timeout = 200;
                timer.reset();
            } else if (state == StateMachine.AUTO_GRAB) {
                intake.stopIntake();
                claw.setState(ClawSubsystem.ClawState.SCORE);
                state = StateMachine.AUTO_RAISE;
                timeout = 200;
                timer.reset();
            } else if (state == StateMachine.AUTO_RAISE) {
                state = StateMachine.IDLE;
            }

            if (stateClawSample == StateMachine.CLAW_SAMPLE) {
                intake.reverseIntake();
                elevator.goToPositionPID(ElevatorSubsystem.ELEVATOR_PRESET_HIGH);
                stateClawSample = StateMachine.DELIVER_SAMPLE;
                timeout = 200;
                timer.reset();
            }
            else if (stateClawSample == StateMachine.DELIVER_SAMPLE) {
                claw.setState(ClawSubsystem.ClawState.SCORE);
                intake.stopIntake();
                stateClawSample = StateMachine.DELIVERY_SPECIMENT;
            }
            else if (stateClawSample == StateMachine.CLAW_RETRACT) {
                claw.setState(ClawSubsystem.ClawState.TRAVEL);
                elevator.goToPositionPID(ElevatorSubsystem.ELEVATOR_PRESET_GROUND);
                intake.stopIntake();
                stateClawSample = StateMachine.CLAW_SPECIMENT;
            }
        }

        // --- ATUALIZAÇÕES FINAIS E TELEMETRIA ---
        elevator.update(telemetry);
        intake.update(telemetry);
        telemetry.addData("Main State", state);
        telemetry.addData("Claw State", stateClawSample);
        telemetry.addData("Pose", follower.getPose());
        telemetry.update();
    }

    private Pose getVisionPose() {
        limelight.updateRobotOrientation(Math.toDegrees(follower.getPose().getHeading()));
        LLResult result = limelight.getLatestResult();

        if (result == null ||!result.isValid()

                | result.getStaleness() > 200) {
            return null;
        }

        Pose3D visionPose3D = result.getBotpose_MT2();
        if (visionPose3D!= null) {

            return new Pose(
                    visionPose3D.getPosition().x * 100,
                    visionPose3D.getPosition().y * 100,
                    visionPose3D.getOrientation().getYaw(AngleUnit.RADIANS)
            );
        }
        return null;
    }
}