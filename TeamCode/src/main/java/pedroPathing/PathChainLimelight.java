package pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Drawing;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import com.bylazar.ftcontrol.panels.Panels;


import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
@TeleOp
public class PathChainLimelight extends OpMode {
    private static final Logger log = LoggerFactory.getLogger(PathChainLimelight.class);
    private Limelight3A limelight;
    LimelightPoseUpdater llUpdater;
    public static double multiplier = 100.0; // converte metros para centímetros
    public static AngleUnit angle = AngleUnit.DEGREES;
    public static boolean isPedroPathing = true;
    public static double PositionX = 0;
    public static double PositionY = 0;
    private FtcDashboard dashboard;
    private Telemetry telemetryA;
    private PathChain pathChain;
    private Follower follower;

    @Override
    public void init() {

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(new Pose());
        follower.startTeleopDrive();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();


        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.update();
        dashboard = FtcDashboard.getInstance();

        while(true){
            Pose initialPose = getLimelightPose();
            if(initialPose!=null){
                follower.setStartingPose(initialPose);
                break;
            }
        }
    }
    public static Pose weightedAveragePose(Pose odoPose, Pose visionPose, double odoWeight) {
        double visionWeight = 1.0 - odoWeight;

        double x = odoPose.getX() * odoWeight + visionPose.getX() * visionWeight;
        double y = odoPose.getY() * odoWeight + visionPose.getY() * visionWeight;

        // Para heading, faz interpolações circulares simples
        double odoHeading = odoPose.getHeading();
        double visionHeading = visionPose.getHeading();

        double delta = angleWrap(visionHeading - odoHeading);
        double heading = odoHeading + delta * visionWeight;

        return new Pose(x, y, heading);
    }

    // Garante que o ângulo esteja entre -π e π
    public static double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
    @Override
    public void start() {

        follower.startTeleopDrive();

    }
    public void plotPoses(Pose cam, Pose track){
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();
        double xCm = cam.getX();
        double yCm = cam.getY();

        // Heading (yaw) em graus → radianos
        double headingRad = Math.toRadians(cam.getHeading());
        fieldOverlay.setFill("green");
        fieldOverlay.fillCircle(xCm, yCm, 10);

        // Desenhar seta indicando direção
        double lineLength = 20;
        double x2 = xCm + Math.cos(headingRad) * lineLength;
        double y2 = yCm + Math.sin(headingRad) * lineLength;
        fieldOverlay.setStroke("black");
        fieldOverlay.strokeLine(xCm, yCm, x2, y2);

        // Exibir dados na telemetria
        packet.put("x CAM", xCm);
        packet.put("y CAM", yCm);
        packet.put("heading° CAM", headingRad);

        xCm = track.getX();
        yCm = track.getY();

        // Heading (yaw) em graus → radianos
        headingRad = Math.toRadians(track.getHeading());
        fieldOverlay.setFill("red");
        fieldOverlay.fillCircle(xCm, yCm, 10);

        // Desenhar seta indicando direção

        x2 = xCm + Math.cos(headingRad) * lineLength;
        y2 = yCm + Math.sin(headingRad) * lineLength;
        fieldOverlay.setStroke("black");
        fieldOverlay.strokeLine(xCm, yCm, x2, y2);

        // Exibir dados na telemetria
        packet.put("x track", xCm);
        packet.put("y track", yCm);
        packet.put("heading° track", headingRad);

        dashboard.sendTelemetryPacket(packet);
    }
    public Pose getLimelightPose(){

        // Obtém os resultados da Limelight
        LLResult result = limelight.getLatestResult();


        if (result != null && result.isValid()) {
            Pose3D botPose = result.getBotpose();
            if (botPose != null) {
                Position pos = botPose.getPosition();
                YawPitchRollAngles ori = botPose.getOrientation();

                // Converter de metros para centímetros
                double xCm = pos.x * 100.0 /2.54;
                double yCm = pos.y * 100.0/2.54;

                // Heading (yaw) em graus → radianos
                double headingRad = ori.getYaw(AngleUnit.RADIANS);
                // Desenhar o robô


                return new Pose(xCm,yCm,headingRad);
            }
        }
        return null;

    }
    private Pose lastPose = new Pose();
    private Timer timer = new Timer();
    @Override
    public void loop() {
        //follower.update();
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y,-gamepad1.left_stick_x,-gamepad1.right_stick_x,gamepad1.right_bumper);

        // Atualiza a odometria
        Pose pose = getLimelightPose();
        if(pose!=null && timer.getElapsedTime()>250){
            timer.resetTimer();
            lastPose = pose;
            follower.setPose(pose);
            follower.updatePose();

        }

        // Atualiza o follower
        follower.update();
        plotPoses(lastPose,follower.poseUpdater.getPose());
    }
}
