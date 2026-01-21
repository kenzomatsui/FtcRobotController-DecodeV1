package org.firstinspires.ftc.teamcode.drive.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.objects.FieldOrientedDrive;
import org.firstinspires.ftc.teamcode.drive.objects.Intake;
import org.firstinspires.ftc.teamcode.drive.objects.PedroPathingMotorController;
import org.firstinspires.ftc.teamcode.drive.objects.PedroPathingShooterController;
import org.firstinspires.ftc.teamcode.drive.objects.ShooterObjBlue;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class ProtTeleOpBlue extends OpMode {
    FieldOrientedDrive fod;
    ShooterObjBlue shooter;
    Intake intake;
    private Follower follower;
    private PedroPathingMotorController turretController = new PedroPathingMotorController();
    private PedroPathingShooterController shooterController = new PedroPathingShooterController();


    // Nomes de configuração
    private static final String MOTOR_NAME = "RMX";
    private static final String SHOOTER_MOTOR = "RMTa"; // Ajuste conforme seu hardware


    // Alvo inicial: gol azul
    private double targetX = 0;
    private double targetY = 116;

    public void init() {
        fod = new FieldOrientedDrive(hardwareMap);
        shooter = new ShooterObjBlue(hardwareMap);
        intake = new Intake(hardwareMap);

        // 1. Inicializar Pedro Pathing Follower
        // Certifique-se de que sua classe Constants está configurada corretamente
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(7, 7, 0)); // Ajuste conforme sua posição inicial

        // 2. Inicializar Controlador da Turret
        turretController.init(hardwareMap, follower, MOTOR_NAME);
        turretController.setTargetPosition(targetX, targetY);
        turretController.setLimits(0.0, 180.0);

        // 3. Inicializar Shooter
        shooterController.init(hardwareMap, follower, SHOOTER_MOTOR);
        shooterController.setTargetPosition(targetX, targetY);
        // Configura: MinPower 0.35 a 24pol, MaxPower 0.9 a 120pol
        shooterController.setPowerConfig(0.25, 0.90, 28.0, 120.0);

        telemetry.addData("Status", "Inicializado. Pedro Pathing Ativo.");
        telemetry.update();
    }

    public void loop() {
        fod.movement(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_bumper);
        //shooter.aimAndShoot();
        shooter.Shoot(gamepad1.right_trigger);
        intake.Coleta(-gamepad1.left_trigger, -gamepad1.right_trigger);
        follower.update();
        turretController.update();
        shooterController.update();
        //telemetry.addData("Power: ", shooter.get_power());
        telemetry.update();
    }
}

/* CONFIGURAÇÃO DO DRIVER
 * Control hub:
 * 0 - RMX
 * 1 - index
 * 2 - RMTa
 * 3 - intake
 *
 * I2c:
 * 0 - imu
 * 1 - pinpoint
 * 2 - sensor_distance
 *
 * USB 2.0: limelight
 *
 * Expansion hub:
 * 0 - BL
 * 1 - BR
 * 2 - FL
 * 3 - FR
 */
