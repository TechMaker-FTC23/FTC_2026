package pedroPathing.examples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp
public class LimelightComImu extends LinearOpMode {
    private Telemetry telemetryA;

    private Limelight3A limelight;
    private IMU imu;
    private Pose3D botPoseCam;
    private Follower follower;
    private FtcDashboard dashboard;
    private final Pose startPose = new Pose(0,0,0);
    @Override
    public void runOpMode() throws InterruptedException {


        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.update();
        dashboard = FtcDashboard.getInstance();
        imu = hardwareMap.get(IMU.class, "imu");
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        // 2. Inicializar a Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetryA.addData("Status", "Inicialização Completa");
        telemetryA.addData("Info", "Pressione Play para começar a ler os dados");
        telemetryA.update();

        waitForStart();
        follower.startTeleopDrive();

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            double currentYawDegrees = orientation.getYaw(AngleUnit.DEGREES);
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);

            limelight.updateRobotOrientation(currentYawDegrees);


            LLResult result = limelight.getLatestResult();

            Pose3D botpose = null;


            if (result != null && result.isValid()) {
                botpose = result.getBotpose_MT2();
            }


            telemetryA.addData("IMU Yaw (Heading)", "%.2f Graus", currentYawDegrees);

            if (botpose != null) {
                // O botpose retorna as coordenadas em metros.
                telemetryA.addData("Botpose X (metros)", "%.2f", botpose.getPosition().x);
                telemetryA.addData("Botpose Y (metros)", "%.2f", botpose.getPosition().y);
                telemetryA.addData("Botpose Z (metros)", "%.2f", botpose.getPosition().z);

                telemetryA.addData("Botpose Yaw (graus)", "%.2f", botpose.getOrientation().getYaw(AngleUnit.DEGREES));

                // Converte a Pose3D da Limelight (metros) para a Pose 2D do Pedro Pathing (CM)
                Pose visionPose = new Pose(
                        botpose.getPosition().x * 100.0,
                        botpose.getPosition().y * 100.0,
                        botpose.getOrientation().getYaw(AngleUnit.DEGREES)

                );

                follower.setPose(visionPose);

                Pose pedroPose = follower.getPose();
                telemetryA.addData("Pedro pose X (metros)", "%.2f", pedroPose.getX());
                telemetryA.addData("Pedro pose Y (metros)", "%.2f", pedroPose.getY());
                botPoseCam = botpose;
            } else {
                telemetryA.addData("Botpose", "Nenhuma pose válida detectada");
                botPoseCam = new Pose3D(new Position(), new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0));
            }

            // Converter de metros para centímetros
            double xCm = follower.getPose().getX() / 2.54;
            double yCm = follower.getPose().getY() / 2.54;

            // Heading (yaw) em graus → radianos
            double headingRad = follower.getPose().getHeading();

            // Desenhar o robô
            fieldOverlay.setFill("green");
            fieldOverlay.fillCircle(xCm, yCm, 10);

            // Desenhar seta indicando direção
            double lineLength = 20;
            double x2 = xCm + Math.cos(headingRad) * lineLength;
            double y2 = yCm + Math.sin(headingRad) * lineLength;
            fieldOverlay.setStroke("black");
            fieldOverlay.strokeLine(xCm, yCm, x2, y2);

            // Exibir dados na telemetria
            packet.put("x", xCm);
            packet.put("y", yCm);
            packet.put("heading°", follower.getPose().getHeading());
            dashboard.sendTelemetryPacket(packet);
            telemetryA.update();
        }
        limelight.stop();
    }
    public Pose3D getBotPoseCam() {
        return botPoseCam;
    }
}