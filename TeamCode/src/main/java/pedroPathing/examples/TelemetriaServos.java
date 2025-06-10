package org.firstinspires.ftc.teamcode; // Substitua pelo seu pacote

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TelemetriaServos extends OpMode {

    // Declare os objetos Servo
    private Servo servoWrist1;
    private Servo servoWrist2; // Corrigido de "Whirts 2" para "Wrist2"
    private Servo servoArm1;
    private Servo servoArm2;
    private Servo servoClaw;

    @Override
    public void init() {
        // Inicialize cada servo a partir do hardwareMap
        // Certifique-se de que os nomes correspondem à sua configuração no robô
        try {
            servoWrist1 = hardwareMap.get(Servo.class, "wrist1_servo");
            servoWrist2 = hardwareMap.get(Servo.class, "wrist2_servo");
            servoArm1   = hardwareMap.get(Servo.class, "arm1_servo");
            servoArm2   = hardwareMap.get(Servo.class, "arm2_servo");
            servoClaw   = hardwareMap.get(Servo.class, "claw_servo");

            telemetry.addData("Status", "Servos Inicializados");
        } catch (Exception e) {
            telemetry.addData("Erro", "Não foi possível inicializar um ou mais servos: " + e.getMessage());
            // Define os servos como null para evitar NullPointerExceptions no loop se a inicialização falhar
            servoWrist1 = null;
            servoWrist2 = null;
            servoArm1 = null;
            servoArm2 = null;
            servoClaw = null;
        }
        telemetry.update();
    }

    @Override
    public void loop() {
        // Envie a posição atual de cada servo para a telemetria
        // O método getPosition() retorna um valor entre 0.0 e 1.0

        if (servoWrist1!= null) {
            telemetry.addData("Wrist 1 Posição", "%.2f", servoWrist1.getPosition());
        } else {
            telemetry.addData("Wrist 1", "Não inicializado");
        }

        if (servoWrist2!= null) {
            telemetry.addData("Wrist 2 Posição", "%.2f", servoWrist2.getPosition());
        } else {
            telemetry.addData("Wrist 2", "Não inicializado");
        }

        if (servoArm1!= null) {
            telemetry.addData("Arm 1 Posição", "%.2f", servoArm1.getPosition());
        } else {
            telemetry.addData("Arm 1", "Não inicializado");
        }

        if (servoArm2!= null) {
            telemetry.addData("Arm 2 Posição", "%.2f", servoArm2.getPosition());
        } else {
            telemetry.addData("Arm 2", "Não inicializado");
        }

        if (servoClaw!= null) {
            telemetry.addData("Claw Posição", "%.2f", servoClaw.getPosition());
        } else {
            telemetry.addData("Claw", "Não inicializado");
        }

        telemetry.update(); // Atualiza a telemetria na Driver Station
    }
}