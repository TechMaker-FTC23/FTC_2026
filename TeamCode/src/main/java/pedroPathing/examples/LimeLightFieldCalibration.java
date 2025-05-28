package pedroPathing.examples;

import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name="Calibrar Limelight → Campo", group="Calibração")
public class LimeLightFieldCalibration extends OpMode {
    private Limelight3A limelight;
    private static final int N_POINTS = 2;

    // 1) Defina aqui os pontos conhecidos de campo (X, Y, heading) em suas unidades finais:
    //    ex: metros ou centímetros, conforme seu grid.
    private final Pose[] knownFieldPoses = {
            new Pose(  0.0,   0.0, 0.0),   // ponto 1: (ex: centro do campo)
            new Pose(100.0,   0.0, 0.0)    // ponto 2: 1 m à frente, por exemplo
    };

    private double[] measX = new double[N_POINTS];
    private double[] measY = new double[N_POINTS];

    private int   index = 0;
    private boolean waitingForPress = true;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addLine("CALIBRAÇÃO DE VISÃO");
        telemetry.addLine("Posicione robô no primeiro ponto conhecido e aperte Ⓐ");
        telemetry.update();
    }

    @Override
    public void loop() {
        LLResult r = limelight.getLatestResult();
        if (r != null && r.isValid()) {
            Pose3D bp = r.getBotpose_MT2();

            if (gamepad1.a && waitingForPress) {
                // captura medição
                measX[index] = bp.getPosition().x;
                measY[index] = bp.getPosition().y;
                telemetry.addData(
                        "Capturado P" + (index+1) + " (raw)",
                        String.format("x=%.3f m, y=%.3f m", measX[index], measY[index])
                );
                index++;
                waitingForPress = false;

                if (index < N_POINTS) {
                    telemetry.addLine("");
                    telemetry.addLine("Agora posicione no próximo ponto e aperte Ⓐ");
                }
            }
            // espera o botão ser liberado
            if (!gamepad1.a) {
                waitingForPress = true;
            }

            // quando terminar de coletar:
            if (index == N_POINTS) {
                // resolve escala e offset em X e Y:
                double dxRaw = measX[1] - measX[0];
                double dyRaw = measY[1] - measY[0];
                double dxTrue = knownFieldPoses[1].getX() - knownFieldPoses[0].getX();
                double dyTrue = knownFieldPoses[1].getY() - knownFieldPoses[0].getY();

                double scaleX = dxTrue / dxRaw;
                double scaleY = dyTrue / dyRaw;

                double offsetX = knownFieldPoses[0].getX() - measX[0] * scaleX;
                double offsetY = knownFieldPoses[0].getY() - measY[0] * scaleY;

                telemetry.clearAll();
                telemetry.addLine(">>> CALIBRAÇÃO COMPLETA <<<");
                telemetry.addData("multiplierX", "%.4f", scaleX);
                telemetry.addData("multiplierY", "%.4f", scaleY);
                telemetry.addData("offsetX",     "%.4f", offsetX);
                telemetry.addData("offsetY",     "%.4f", offsetY);
                telemetry.addLine("");
                telemetry.addLine("Copie esses valores no seu OpMode principal.");
                telemetry.update();

                // trava loop para não recalcular
                index++;
            }
        } else {
            telemetry.addLine("Sem visão (verifique pipeline / tag)");
            telemetry.update();
        }
    }
}