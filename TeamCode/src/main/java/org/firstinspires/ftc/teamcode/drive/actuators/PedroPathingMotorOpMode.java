package org.firstinspires.ftc.teamcode.drive.actuators;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * OpMode TeleOp utilizando Pedro Pathing para localização e controle de base giratória.
 * Pressione 'A' no Gamepad 1 para definir a posição atual do robô como o alvo.
 */
@TeleOp(name = "Pedro Pathing Turret Control", group = "TeamCode")
public class PedroPathingMotorOpMode extends LinearOpMode {

    private Follower follower;
    private PedroPathingMotorController turretController = new PedroPathingMotorController();

    // Nomes de configuração
    private static final String MOTOR_NAME = "RMX";

    // Alvo inicial (Pedro Pathing usa polegadas, 72,72 é o centro do campo)
    private double targetX = 0;
    private double targetY = 116;
    private boolean aButtonPreviousState = false;

    @Override
    public void runOpMode() {
        // 1. Inicializar Pedro Pathing Follower
        // Certifique-se de que sua classe Constants está configurada corretamente
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(7, 7, 0)); // Ajuste conforme sua posição inicial

        // 2. Inicializar Controlador da Turret
        turretController.init(hardwareMap, follower, MOTOR_NAME);
        turretController.setTargetPosition(targetX, targetY);
        turretController.setLimits(-150.0, 150.0);
        turretController.setAutoReverse(true);

        telemetry.addData("Status", "Inicializado. Pedro Pathing Ativo.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Atualiza o Follower (Essencial para a odometria do Pedro Pathing)
            follower.update();

            // Lógica de captura de alvo com botão 'A'
            if (gamepad1.a && !aButtonPreviousState) {
                Pose currentPose = turretController.getCurrentPose();
                targetX = currentPose.getX();
                targetY = currentPose.getY();
                turretController.setTargetPosition(targetX, targetY);
            }
            aButtonPreviousState = gamepad1.a;

            // Atualiza a lógica de apontamento da turret
            turretController.update();

            // Telemetria
            telemetry.addData("--- Pedro Pathing Pose ---", "");
            telemetry.addData("X (in)", "%.2f", follower.getPose().getX());
            telemetry.addData("Y (in)", "%.2f", follower.getPose().getY());
            telemetry.addData("Heading (deg)", "%.2f", Math.toDegrees(follower.getPose().getHeading()));

            telemetry.addData("--- Turret Target ---", "");
            telemetry.addData("Target X", "%.1f", targetX);
            telemetry.addData("Target Y", "%.1f", targetY);

            telemetry.addData("--- Turret Status ---", "");
            telemetry.addData("Ângulo Motor", "%.1f°", turretController.getMotorAngle());
            telemetry.addData("Poder", "%.2f", turretController.getMotorPower());

            telemetry.update();
        }
    }
}
