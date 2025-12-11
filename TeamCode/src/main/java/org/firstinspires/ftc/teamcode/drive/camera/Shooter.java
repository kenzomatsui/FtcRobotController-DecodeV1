package org.firstinspires.ftc.teamcode.drive.camera;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.objects.ShooterObj;

@Disabled
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

            shooter.aimAndShoot(); // Faz o alinhamento e o disparo com base na Limelight

            telemetry.addData("Distância", shooter.getDistance());
            telemetry.update();
        }
    }
}