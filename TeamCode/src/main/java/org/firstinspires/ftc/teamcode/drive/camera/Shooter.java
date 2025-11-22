package org.firstinspires.ftc.teamcode.drive.camera;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.objects.ShooterObj;

@TeleOp(name = "ShooterPOO", group = "Drive")
public class Shooter extends LinearOpMode {

    private ShooterObj shooter; // Objeto da classe Shooter

    @Override
    public void runOpMode() {
        shooter = new ShooterObj(hardwareMap); // Inicializa o sistema

        telemetry.addData("Status", "Inicializado");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            shooter.update(); // Atualiza o shooter a cada ciclo
            shooter.detectBall(gamepad1.right_trigger); // Verifica bola e controla Indexer
            shooter.aimAndShoot(); // Faz o alinhamento e o disparo com base na Limelight

            if (shooter.ACTUAL_SPEED >= shooter.TARGET_SPEED){
                gamepad1.rumble(300);
            }
            telemetry.addData("Distância", shooter.getDistance());
            telemetry.addData("Target: ", shooter.TARGET_SPEED);
            telemetry.addData("Actual: ", shooter.ACTUAL_SPEED);
            telemetry.update();
        }
    }
}