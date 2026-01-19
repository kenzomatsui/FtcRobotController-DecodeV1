package org.firstinspires.ftc.teamcode.drive.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.objects.FieldOrientedDrive;
import org.firstinspires.ftc.teamcode.drive.objects.Intake;
import org.firstinspires.ftc.teamcode.drive.objects.PedroPathingMotorController;
import org.firstinspires.ftc.teamcode.drive.objects.ShooterObjBlue;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class ProtTeleOpBlue extends OpMode {
    FieldOrientedDrive fod;
    ShooterObjBlue shooter;
    Intake intake;
    private Follower follower;
    private PedroPathingMotorController turretController = new PedroPathingMotorController();

    // Nomes de configuração
    private static final String MOTOR_NAME = "RMX";

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
        turretController.setLimits(-150.0, 150.0);
        turretController.setAutoReverse(true);

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
