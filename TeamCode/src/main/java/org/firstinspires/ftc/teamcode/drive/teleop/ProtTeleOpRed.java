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
import org.firstinspires.ftc.teamcode.pedroPathingVelho.Constants;

@TeleOp
public class ProtTeleOpRed extends OpMode {
    FieldOrientedDrive fod;
    ShooterObjBlue shooter;
    Intake intake;

    private Follower follower;
    private PedroPathingMotorController turretController = new PedroPathingMotorController();
    private PedroPathingShooterController shooterController = new PedroPathingShooterController();
    private final Pose startTeleop = new Pose(105, 80, Math.toRadians(180)); //pose pro inicio do teleop vermelho com o intake vira pra spike mark



    // Nomes de configuração
    private static final String MOTOR_NAME = "RMX";
    private static final String SHOOTER_MOTOR = "RMTa"; // Ajuste conforme seu hardware

    private static int counter = 0;


    // Alvo inicial: gol azul
    private double targetX = -138; // 6 é o verdadeiro (pro vermelho é 138 aqui)
    private double targetY = -138; //138 é o verdadeiro

    public void init() {
        fod = new FieldOrientedDrive(hardwareMap);
        shooter = new ShooterObjBlue(hardwareMap);
        intake = new Intake(hardwareMap);

        // 1. Inicializar Pedro Pathing Follower
        // Certifique-se de que sua classe Constants está configurada corretamente
        follower = Constants.createFollower(hardwareMap);// Ajuste conforme sua posição inicial
        follower.setPose(startTeleop);

        // 2. Inicializar Controlador da Turret
        turretController.init(hardwareMap, follower, MOTOR_NAME);
        turretController.setTargetPosition(targetX, targetY);
        turretController.setLimits(-60.0, 260.0);

        // 3. Inicializar Shooter
        shooterController.init(hardwareMap, follower, SHOOTER_MOTOR);
        shooterController.setTargetPosition(targetX, targetY);
        // Configura: MinPower 0.35 a 24pol, MaxPower 0.9 a 120pol
        shooterController.setPowerConfig(0.35, 0.95, 20.0, 110.0);

        telemetry.addData("Status", "Inicializado. Pedro Pathing Ativo.");
        telemetry.update();
    }

    public void loop() {
        follower.update();
        fod.movement(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_bumper);
        shooter.Shoot(gamepad1.right_trigger);
        shooter.SHOOTER3(gamepad1.a);

        intake.Coleta(-gamepad1.left_trigger, -gamepad1.right_trigger);

        if (gamepad1.b && counter == 0){
            follower.setPose(startTeleop);
            counter++;
        }

        turretController.update();
        shooterController.update();
        telemetry.addData("Power: ", shooterController.getCurrentPower()); //Alteração
        telemetry.addData("Angle: ", turretController.getMotorAngle());
        telemetry.addData("Pose X: ", follower.getPose().getX());
        telemetry.addData("Pose Y: ", follower.getPose().getY());

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
