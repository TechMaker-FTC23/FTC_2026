package techmaker.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;

import techmaker.Constants;
import techmaker.Constants.Claw.*;
@Config
// ALTERADO: O nome da classe agora descreve melhor sua função completa.
public class ClawSubsystem {

    // --- Constantes ---

    // NOVO: Constantes para os nomes do hardware. Evita erros de digitação e facilita a manutenção.
    public static double maxClaw = 0.65;
    public static double minClaw = 0.4;
    public static double maxArmL = 0.12;
    public static double minArmL = 0.45;
    public static double maxArmR = 0.88;
    public static double minArmR = 0.55;
    public static double medArmR = 0.2;
    public static double medArml = 0.8;
    public static double medWristR = 0.2;
    public static double medWristl = 0.8;
    public static double maxWristL = 0.65;
    public static double minWristL = 0.7;
    public static double maxWristR = 0.35;
    public static double minWristR = 0.3;


    // --- Declarações de Hardware ---
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
        middleClawServo = hardwareMap.get(Servo.class, Constants.Claw.MiddleClaw);
        leftClawWrist = hardwareMap.get(Servo.class, Constants.Claw.LeftClaw);
        rightClawWrist = hardwareMap.get(Servo.class, Constants.Claw.RightClaw);
        leftClawArm = hardwareMap.get(Servo.class, Constants.Claw.LeftArm);
        rightClawArm = hardwareMap.get(Servo.class, Constants.Claw.RightArm);
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