package pedroPathing;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;


public class LimelightPoseUpdater {
    private final Limelight3A limelight;
    private final PoseUpdater localizer;
    private final Telemetry telemetry;
    private final Timer timer = new Timer();
    public LimelightPoseUpdater(Limelight3A limelight, PoseUpdater localizer, Telemetry telemetry) {
        this.limelight = limelight;
        this.localizer = localizer;
        this.telemetry = telemetry;
        timer.resetTimer();


    }

    public void updatePose() {
        if(timer.getElapsedTime()> 1) { // Atualiza a pose a cada 100ms
            timer.resetTimer();
        } else {
            return; // Se não passou o tempo, não atualiza
        }
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botPose = result.getBotpose();
            if (botPose != null) {
                // Extrair coordenadas em metros
                double xMeters = botPose.getPosition().x;
                double yMeters = botPose.getPosition().y;
                double yawDegrees = botPose.getOrientation().getYaw(AngleUnit.DEGREES);

                // Converter para centímetros e radianos
                double xCm = xMeters * 100.0;
                double yCm = yMeters * 100.0;
                double headingRad = Math.toRadians(yawDegrees);

                // Criar nova pose e atualizar o localizador
                // Offset: centro da arena para canto inferior esquerdo
                double offsetX = -183.0; // cm
                double offsetY = -183.0; // cm

                Pose poseLimelight = new Pose(xCm, yCm, headingRad);
                Pose poseAjustada = new Pose(
                        poseLimelight.getX() + offsetX,
                        poseLimelight.getY() + offsetY,
                        poseLimelight.getHeading()
                );

                telemetry.addData("Pose limelight",poseLimelight);
                telemetry.addData("Pose Ajustada",poseAjustada);
                localizer.setPose(poseLimelight);
            }
        }
    }
}
