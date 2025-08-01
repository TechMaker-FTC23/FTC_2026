package techmaker.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;

@Config
public class ClawSubsystem {

    public static final String MIDDLE_CLAW_SERVO_NAME = "middleclaw";
    public static final String LEFT_CLAW_WRIST_NAME = "leftclaw";
    public static final String RIGHT_CLAW_WRIST_NAME = "rightclaw";
    public static final String LEFT_CLAW_ARM_NAME = "leftclawarm";
    public static final String RIGHT_CLAW_ARM_NAME = "rightclawarm";
    public static double maxClaw = 0.65;
    public static double minClaw = 0.4;
    public static double maxArmL = 0.12;
    public static double minArmL = 0.45;
    public static double maxArmR = 0.88;
    public static double minArmR = 0.55;
    public static double medArmR = 0.2;
    public static double medArml = 0.8;
    public static double medWristR = 0.24;
    public static double medWristl = 0.76;
    public static double maxWristL = 0.65;
    public static double minWristL = 0.7;
    public static double maxWristR = 0.35;
    public static double minWristR = 0.3;

    private final Servo middleClawServo;
    private final Servo leftClawWrist;
    private final Servo rightClawWrist;
    private final Servo leftClawArm;
    private final Servo rightClawArm;
    private final ElapsedTime timer = new ElapsedTime();
    /**
     * Construtor do subsistema unificado do efetor final.
     * @param hardwareMap O mapa de hardware do robô.
     */
    public ClawSubsystem(HardwareMap hardwareMap) {
        middleClawServo = hardwareMap.get(Servo.class, MIDDLE_CLAW_SERVO_NAME);
        leftClawWrist = hardwareMap.get(Servo.class, LEFT_CLAW_WRIST_NAME);
        rightClawWrist = hardwareMap.get(Servo.class, RIGHT_CLAW_WRIST_NAME);
        leftClawArm = hardwareMap.get(Servo.class, LEFT_CLAW_ARM_NAME);
        rightClawArm = hardwareMap.get(Servo.class, RIGHT_CLAW_ARM_NAME);
        // Configuração inicial dos mecanismos

        timer.reset();

    }
    public void update(Telemetry telemetry){
        if(timer.time(TimeUnit.MILLISECONDS)>20){
            timer.reset();
            telemetry.addData("middle",middleClawServo.getPosition());
            telemetry.addData("leftClaw",leftClawWrist.getPosition());
            telemetry.addData("rightClaw",rightClawWrist.getPosition());
            telemetry.addData("leftArm",leftClawArm.getPosition());
            telemetry.addData("rightArm",rightClawArm.getPosition());
        }
    }
   public void middleClaw(double position) {
        middleClawServo.setPosition(position);
   }
    public void clawWrist(double positionL, double positionR) {
        leftClawWrist.setPosition(positionL);
        rightClawWrist.setPosition(positionR);
   }
    public void clawArm(double positionL, double positionR) {
        leftClawArm.setPosition(positionL);
        rightClawArm.setPosition(positionR);
   }



}