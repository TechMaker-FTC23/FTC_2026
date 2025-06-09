import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous
public class ServoTest extends OpMode {
   private Servo s0;
   private CRServo s5;
   public static double s0Position = 0; // Posição inicial do servo s0
    public static double s5Position = 0; // Posição inicial do servo s5
    private double lastS0, lastS5;
    @Override
    public void init() {
        s0 = hardwareMap.get(Servo.class, "s0");
        s5 = hardwareMap.get(CRServo.class, "s5");

    }

    @Override
    public void loop() {
        if(s0Position != lastS0) {
            s0.setPosition(s0Position ); // Converte de porcentagem para posição do servo
            lastS0 = s0Position;
        }
        if(s5Position != lastS5) {
            s5.setPower (s5Position); // Converte de porcentagem para posição do servo
            lastS5 = s5Position;
        }


    }
}
