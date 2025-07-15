// Em pedroPathing/util/KalmanFilter2D.java
package pedroPathing.util;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;

@Config
public class KalmanFilter2D {

    public static double Q_PROCESS_VARIANCE = 0.01;
    public static double R_MEASUREMENT_VARIANCE = 0.2;

    private Pose state;
    private double pX = 1.0, pY = 1.0, pH = 1.0;
    private long lastUpdateTime;

    // NOVO: Variável para rastrear a última pose da odometria
    private Pose lastOdometryPose;

    public KalmanFilter2D(Pose startPose) {
        this.state = startPose;
        this.lastOdometryPose = startPose; // Inicializa com a mesma pose
        this.lastUpdateTime = System.nanoTime();
    }

    public void predict(Pose currentOdometryPose) {
        // Calcula a mudança (delta) na odometria desde a última atualização
        double dx = currentOdometryPose.getX() - lastOdometryPose.getX();
        double dy = currentOdometryPose.getY() - lastOdometryPose.getY();
        double dHeading = currentOdometryPose.getHeading() - lastOdometryPose.getHeading();

        // Normaliza a mudança de heading
        while (dHeading > Math.PI) dHeading -= 2 * Math.PI;
        while (dHeading < -Math.PI) dHeading += 2 * Math.PI;

        // Aplica a mudança à nossa última estimativa FUNDIDA (não à odometria com drift)
        state = new Pose(
                state.getX() + dx,
                state.getY() + dy,
                state.getHeading() + dHeading
        );

        // Atualiza a última pose da odometria para o próximo ciclo
        lastOdometryPose = currentOdometryPose;

        // Aumenta a incerteza com base no tempo
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastUpdateTime) / 1.0e9;
        lastUpdateTime = currentTime;

        pX += Q_PROCESS_VARIANCE * dt;
        pY += Q_PROCESS_VARIANCE * dt;
        pH += Q_PROCESS_VARIANCE * dt;
    }

    // O método update() permanece o mesmo
    public void update(Pose visionPose) {
        double kX = pX / (pX + R_MEASUREMENT_VARIANCE);
        double kY = pY / (pY + R_MEASUREMENT_VARIANCE);
        double kH = pH / (pH + R_MEASUREMENT_VARIANCE);

        double fusedX = state.getX() + kX * (visionPose.getX() - state.getX());
        double fusedY = state.getY() + kY * (visionPose.getY() - state.getY());

        double headingError = visionPose.getHeading() - state.getHeading();
        while (headingError > Math.PI) headingError -= 2 * Math.PI;
        while (headingError < -Math.PI) headingError += 2 * Math.PI;
        double fusedHeading = state.getHeading() + kH * headingError;

        state = new Pose(fusedX, fusedY, fusedHeading);

        pX *= (1 - kX);
        pY *= (1 - kY);
        pH *= (1 - kH);
    }

    public Pose getEstimate() {
        return state;
    }
}