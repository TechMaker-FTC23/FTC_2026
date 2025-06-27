
package pedroPathing.util;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;

@Config // Permite ajustar os pesos pelo FTC Dashboard
public class KalmanFilter2D {

    // --- Constantes de Tuning (Ajuste estes valores!) ---
    /**
     * Variância do Processo (Q): Quão incerta nossa odometria se torna a cada segundo.
     * Um valor maior significa que confiamos menos na odometria e mais na visão.
     * Comece com um valor pequeno.
     */
    public static double Q_PROCESS_VARIANCE = 0.01;

    /**
     * Variância da Medição (R): Quão "ruidosa" ou incerta é a nossa medição da Limelight.
     * Um valor maior significa que confiamos menos na Limelight.
     * Comece com um valor maior que Q.
     */
    public static double R_MEASUREMENT_VARIANCE = 0.2;

    // Estado atual e incerteza (covariância)
    private Pose state;
    private double pX = 1.0, pY = 1.0, pH = 1.0; // Incerteza inicial
    private long lastPredictTime;

    public KalmanFilter2D(Pose startPose) {
        this.state = startPose;
        this.lastPredictTime = System.nanoTime();
    }

    /**
     * Etapa de Predição: Usa a odometria para prever a nova posição.
     * @param odometryPose A nova pose completa calculada pelo localizador (OTOS/IMU).
     */
    public void predict(Pose odometryPose) {
        // A odometria do Pedro Pathing já faz a predição para nós.
        // Nós simplesmente adotamos a nova pose da odometria como nossa nova predição.
        if (odometryPose.getX() == 0 && odometryPose.getY() == 0 && odometryPose.getHeading() == 0) {
            return;
        }
        this.state = odometryPose;

        // Aumentamos a incerteza com base no tempo decorrido desde a última predição.
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastPredictTime) / 1.0e9; // Delta de tempo em segundos
        lastPredictTime = currentTime;

        // A incerteza cresce com o tempo.
        pX += Q_PROCESS_VARIANCE * dt;
        pY += Q_PROCESS_VARIANCE * dt;
        pH += Q_PROCESS_VARIANCE * dt;
    }

    /**
     * Etapa de Atualização: Corrige a predição usando a medição da visão.
     * @param visionPose A pose medida pela Limelight.
     */
    public void update(Pose visionPose) {
        // Calcula o Ganho de Kalman para cada eixo
        double kX = pX / (pX + R_MEASUREMENT_VARIANCE);
        double kY = pY / (pY + R_MEASUREMENT_VARIANCE);
        double kH = pH / (pH + R_MEASUREMENT_VARIANCE);

        // Atualiza o estado (a estimativa de pose)
        double fusedX = state.getX() + kX * (visionPose.getX() - state.getX());
        double fusedY = state.getY() + kY * (visionPose.getY() - state.getY());

        // --- CORREÇÃO PARA ÂNGULOS ---
        double headingError = visionPose.getHeading() - state.getHeading();
        // Normaliza o erro para o intervalo [-PI, PI] para encontrar o caminho mais curto
        while (headingError > Math.PI) headingError -= 2 * Math.PI;
        while (headingError < -Math.PI) headingError += 2 * Math.PI;
        double fusedHeading = state.getHeading() + kH * headingError;
        // --- FIM DA CORREÇÃO ---

        state = new Pose(fusedX, fusedY, fusedHeading);

        // Atualiza a incerteza (diminui, pois acabamos de receber uma medição)
        pX *= (1 - kX);
        pY *= (1 - kY);
        pH *= (1 - kH);
    }

    /**
     * Retorna a pose final, fundida.
     */
    public Pose getEstimate() {
        return state;
    }
}