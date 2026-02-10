package org.firstinspires.ftc.teamcode.drive.actuators;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.drive.objects.PedroPathingMotorController;
import org.firstinspires.ftc.teamcode.drive.objects.PedroPathingShooterController;
import org.firstinspires.ftc.teamcode.pedroPathingVelho.Constants;

/**
 * OpMode TeleOp integrando Turret (Base Giratória) e Shooter dinâmico.
 * Ambos utilizam a odometria do Pedro Pathing para se orientar pelo campo.
 */
@TeleOp(name = "Pedro Pathing Turret & Shooter", group = "TeamCode")
public class PedroPathingFullControlOpMode extends LinearOpMode {

    private Follower follower;
    private PedroPathingMotorController turretController = new PedroPathingMotorController();
    private PedroPathingShooterController shooterController = new PedroPathingShooterController();

    // Nomes de configuração
    private static final String TURRET_MOTOR = "RMX";
    private static final String SHOOTER_MOTOR = "RMTa"; // Ajuste conforme seu hardware

    // Alvo inicial (Ex: Centro do campo 72,72)
    private double targetX = 0;
    private double targetY = 116;

    @Override
    public void runOpMode() {
        // 1. Inicializar Pedro Pathing Follower
        // Nota: Use o método de inicialização correto da sua versão do Pedro Pathing
        // Se 'new Follower(hardwareMap)' der erro, use sua classe de Constants.
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(7, 7, 0));

        // 2. Inicializar Turret
        turretController.init(hardwareMap, follower, TURRET_MOTOR);
        turretController.setTargetPosition(targetX, targetY);
        turretController.setLimits(0, 180);

        // 3. Inicializar Shooter
        shooterController.init(hardwareMap, follower, SHOOTER_MOTOR);
        shooterController.setTargetPosition(targetX, targetY);
        // Configura: MinPower 0.35 a 24pol, MaxPower 0.9 a 120pol
        shooterController.setPowerConfig(0.35, 0.90, 24.0, 120.0);

        telemetry.addData("Status", "Inicializado. Pronto para o combate.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            // Atualiza ambos os controladores
            turretController.update();
            shooterController.update();

            // Telemetria combinada
            telemetry.addData("--- Alvo ---", "X: %.1f, Y: %.1f", targetX, targetY);
            telemetry.addData("Distância ao Alvo", "%.1f pol", shooterController.getDistance());

            telemetry.addData("--- Turret ---", "");
            telemetry.addData("Ângulo", "%.1f°", turretController.getMotorAngle());

            telemetry.addData("--- Shooter ---", "");
            telemetry.addData("Potência Atual", "%.2f", shooterController.getCurrentPower());

            telemetry.update();
        }
    }
}
