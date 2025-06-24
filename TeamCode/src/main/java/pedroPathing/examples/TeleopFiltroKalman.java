package pedroPathing.examples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
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

@Config
@TeleOp
public class TeleopFiltroKalman extends LinearOpMode {
    private Telemetry telemetryA;
    private Limelight3A limelight;
    private IMU imu;
    private Pose3D botPoseCam;
    private Follower follower;
    private FtcDashboard dashboard;
    private final Pose startPose = new Pose(0,0,0);

    private KalmanFilter2D kalmanFilter = new KalmanFilter2D();
    private long lastVisionUpdate = 0;
    private final long visionInterval = 150; // ms

    private double lastTime = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.update();
        dashboard = FtcDashboard.getInstance();
        imu = hardwareMap.get(IMU.class, "imu");
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetryA.addData("Status", "Inicialização Completa");
        telemetryA.addData("Info", "Pressione Play para começar a ler os dados");
        telemetryA.update();

        kalmanFilter.initialize(0, 0, 0);
        lastTime = getRuntime();

        waitForStart();
        while (opModeIsActive()) {
            double currentTime = getRuntime();
            double dt = currentTime - lastTime;
            lastTime = currentTime;

            double forward = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double rotate = -gamepad1.right_stick_x;

            double velocityScale = 60.0; // cm/s
            double dx = forward * velocityScale * dt;
            double dy = strafe * velocityScale * dt;
            double dheading = rotate * Math.PI * dt;

            kalmanFilter.predict(dx, dy, dheading);

            follower.startTeleopDrive();
            follower.setTeleOpMovementVectors(forward, strafe, rotate, true);
            follower.setPose(kalmanFilter.getEstimate());
            follower.update();

            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            double currentYawDegrees = orientation.getYaw(AngleUnit.DEGREES);
            limelight.updateRobotOrientation(currentYawDegrees);

            LLResult result = limelight.getLatestResult();
            long now = System.currentTimeMillis();
            if (result != null && result.isValid() && (now - lastVisionUpdate > visionInterval)) {
                Pose3D botpose = result.getBotpose_MT2();
                Pose visionPose = new Pose(
                        botpose.getPosition().x * 100.0,
                        botpose.getPosition().y * 100.0,
                        botpose.getOrientation().getYaw(AngleUnit.DEGREES)
                );

                Pose estimate = kalmanFilter.getEstimate();
                double dxErr = visionPose.getX() - estimate.getX();
                double dyErr = visionPose.getY() - estimate.getY();
                double mDist2 = dxErr*dxErr / kalmanFilter.pX + dyErr*dyErr / kalmanFilter.pY;
                double threshold = 16.0; // 4 desvios-padrão
                if (mDist2 < threshold) {
                    kalmanFilter.update(visionPose.getX(), visionPose.getY(), visionPose.getHeading());
                    lastVisionUpdate = now;
                    botPoseCam = botpose;
                } else {
                    telemetryA.addData("Filtro Kalman", "Rejeitou visão: dist=%.2f", mDist2);
                }
            } else if (result == null || !result.isValid()) {
                telemetryA.addData("Botpose", "Nenhuma pose válida detectada");
                botPoseCam = new Pose3D(new Position(), new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0));
            }

            Pose fusedPose = kalmanFilter.getEstimate();

            double xCm = fusedPose.getX() / 2.54;
            double yCm = fusedPose.getY() / 2.54;
            double headingRad = fusedPose.getHeading();

            fieldOverlay.setFill("green");
            fieldOverlay.fillCircle(xCm, yCm, 10);

            double lineLength = 20;
            double x2 = xCm + Math.cos(headingRad) * lineLength;
            double y2 = yCm + Math.sin(headingRad) * lineLength;
            fieldOverlay.setStroke("black");
            fieldOverlay.strokeLine(xCm, yCm, x2, y2);

            packet.put("x", xCm);
            packet.put("y", yCm);
            packet.put("heading°", fusedPose.getHeading());

            dashboard.sendTelemetryPacket(packet);
            telemetryA.update();
        }
        limelight.stop();
    }

    public Pose3D getBotPoseCam() {
        return botPoseCam;
    }

    private static class KalmanFilter2D {
        private double x, y, heading;
        private double q = 0.05;
        private double r = 2.5;
        public double pX = 1, pY = 1, pH = 1;

        public void initialize(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }

        public void predict(double dx, double dy, double dheading) {
            x += dx;
            y += dy;
            heading += dheading;
            pX += q;
            pY += q;
            pH += q;
        }

        public void update(double zX, double zY, double zHeading) {
            double kX = pX / (pX + r);
            double kY = pY / (pY + r);
            double kH = pH / (pH + r);

            x += kX * (zX - x);
            y += kY * (zY - y);
            heading += kH * (zHeading - heading);

            pX *= (1 - kX);
            pY *= (1 - kY);
            pH *= (1 - kH);
        }

        public Pose getEstimate() {
            return new Pose(x, y, heading);
        }
    }
}
