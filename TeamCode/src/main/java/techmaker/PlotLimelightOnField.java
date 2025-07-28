package techmaker;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
@Disabled

@TeleOp
public class PlotLimelightOnField extends LinearOpMode {
    private Limelight3A limelight;
    private FtcDashboard dashboard;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(50);
        limelight.start();

        dashboard = FtcDashboard.getInstance();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();

            if (result != null && result.isValid()) {
                Pose3D botPose = result.getBotpose_MT2();
                if (botPose != null) {
                    Position pos = botPose.getPosition();
                    YawPitchRollAngles ori = botPose.getOrientation();

                    // Converter de metros para centímetros
                    double xCm = pos.x * 100.0 /2.54;
                    double yCm = pos.y * 100.0/2.54;

                    // Heading (yaw) em graus → radianos
                    double headingRad = Math.toRadians(ori.getYaw());

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
                    packet.put("heading°", ori.getYaw());
                }
            }

            dashboard.sendTelemetryPacket(packet);
            sleep(75);
        }
    }
}

