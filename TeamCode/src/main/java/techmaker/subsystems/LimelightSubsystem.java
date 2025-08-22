package techmaker.subsystems;

import com.pedropathing.localization.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class LimelightSubsystem {

    private final Limelight3A limelight;

    public LimelightSubsystem(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public Pose updatePoseLimelight(Pose currentPose) {
        double currentYawDegrees = Math.toDegrees(currentPose.getHeading());
        limelight.updateRobotOrientation(currentYawDegrees);

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose_MT2();
            if (botpose != null) {
                // Converte a Pose3D da Limelight (metros) para a Pose 2D do Pedro Pathing (polegadas)
                Pose visionPose = new Pose(
                        botpose.getPosition().x * 39.37, // metros para polegadas
                        botpose.getPosition().y * 39.37, // metros para polegadas
                        botpose.getOrientation().getYaw(AngleUnit.RADIANS)
                );
                return visionPose;
            }
        }
        return currentPose;
    }
}
