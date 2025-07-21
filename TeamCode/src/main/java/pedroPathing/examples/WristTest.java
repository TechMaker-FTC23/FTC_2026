package pedroPathing.examples;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import pedroPathing.subsystems.WristSubsystem;

@TeleOp(name = "Wrist Test OpMode", group = "Test")
public class WristTest extends LinearOpMode {

    private WristSubsystem wristSubsystem;

    @Override
    public void runOpMode() {
        wristSubsystem = new WristSubsystem(hardwareMap);

        telemetry.addLine("Wrist Test OpMode pronto.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                wristSubsystem.openWrist();
            }

            if (gamepad1.x) {
                wristSubsystem.closeWrist();
            }

            if (gamepad1.b) {
                wristSubsystem.setWristMedium();
            }

            wristSubsystem.updateWrist();

            telemetry.addData("Aberto?", wristSubsystem.isWristAberto());
            telemetry.addData("Wrist1", wristSubsystem.getWrist1Position());
            telemetry.addData("Wrist2", wristSubsystem.getWrist2Position());
            telemetry.update();
        }
    }
}
