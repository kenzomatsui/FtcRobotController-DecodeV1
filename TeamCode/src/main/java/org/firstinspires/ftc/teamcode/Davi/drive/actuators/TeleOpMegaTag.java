package org.firstinspires.ftc.teamcode.Davi.drive.actuators;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Davi.drive.objects.FieldOrientedDrive;
import org.firstinspires.ftc.teamcode.Davi.drive.actuators.KalmanFilterLocalizer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * TeleOp de teste para validar a fusão de sensores MegaTag2 usando Limelight3A.
 */
@TeleOp(name = "TeleOp MegaTag2 Limelight3A", group = "TeleOp")
public class TeleOpMegaTag extends OpMode {
    private Follower follower;
    private FieldOrientedDrive fod;
    private KalmanFilterLocalizer kalmanFilter = new KalmanFilterLocalizer();

    @Override
    public void init() {
        // Inicializa o seguidor (Pinpoint via PedroPathing)
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(new Pose(0, 0, 0));

        // Inicializa o Filtro de Kalman com Limelight3A
        kalmanFilter.init(hardwareMap, follower);

        // Drive para movimentar o robô
        fod = new FieldOrientedDrive(hardwareMap);

        telemetry.addLine("Hardware Limelight3A Inicializado.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // 1. Atualiza a fusão de sensores (MegaTag2 + Média Ponderada)
        kalmanFilter.update();

        // 2. Atualiza o seguidor PedroPathing
        follower.update();

        // 3. Controle do Robô
        fod.movement(-gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x, gamepad1.left_bumper);

        // --- TELEMETRIA DE POSIÇÃO ---
        Pose fusedPose = kalmanFilter.getFusedPose();
        Pose visionPose = kalmanFilter.getLimelightPose();

        telemetry.addLine("=== LOCALIZAÇÃO MEGA TAG 2 (3A) ===");
        if (kalmanFilter.hasVision()) {
            telemetry.addData("Vision Status", "CONECTADO (" + kalmanFilter.getTagCount() + " tags)");
            telemetry.addData("Limelight Pose", "X: %.2f, Y: %.2f, H: %.2f",
                    visionPose.getX(), visionPose.getY(), Math.toDegrees(visionPose.getHeading()));
        } else {
            telemetry.addData("Vision Status", "SEM ALVO");
        }

        telemetry.addLine("\n--- POSE FUNDIDA (PINPOINT + 3A) ---");
        telemetry.addData("Pose Final", "X: %.2f, Y: %.2f, H: %.2f",
                fusedPose.getX(), fusedPose.getY(), Math.toDegrees(fusedPose.getHeading()));

        telemetry.update();
    }
}