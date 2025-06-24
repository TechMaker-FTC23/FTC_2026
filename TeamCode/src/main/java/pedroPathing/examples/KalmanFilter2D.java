package pedroPathing.examples;

import com.pedropathing.localization.Pose;

public class KalmanFilter2D {
    private double x, y, heading;
    private double q = 0.02; // Variância do modelo (quanto confia na predição)
    private double r = 1.0;  // Variância da medição (quanto confia no sensor)
    private double pX = 1, pY = 1, pH = 1; // Incerteza inicial

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
