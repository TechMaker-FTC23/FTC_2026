package pedroPathing.examples;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

@TeleOp
public class LimelightComImu extends LinearOpMode {

    private Limelight3A limelight;
    private IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {


        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();

        // 2. Inicializar a Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addData("Status", "Inicialização Completa");
        telemetry.addData("Info", "Pressione Play para começar a ler os dados");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            double currentYawDegrees = orientation.getYaw(AngleUnit.DEGREES);


            limelight.updateRobotOrientation(currentYawDegrees);


            LLResult result = limelight.getLatestResult();

            Pose3D botpose = null;


            if (result!= null && result.isValid()) {

                botpose = result.getBotpose_MT2();
            }


            telemetry.addData("IMU Yaw (Heading)", "%.2f Graus", currentYawDegrees);

            if (botpose!= null) {
                // O botpose retorna as coordenadas em metros.
                telemetry.addData("Botpose X (metros)", "%.2f", botpose.getPosition().x);
                telemetry.addData("Botpose Y (metros)", "%.2f", botpose.getPosition().y);
                telemetry.addData("Botpose Z (metros)", "%.2f", botpose.getPosition().z);

                telemetry.addData("Botpose Yaw (graus)", "%.2f", botpose.getOrientation().getYaw(AngleUnit.DEGREES));
            } else {
                telemetry.addData("Botpose", "Nenhuma pose válida detectada");
            }

            telemetry.update();
        }
        limelight.stop();
    }
}