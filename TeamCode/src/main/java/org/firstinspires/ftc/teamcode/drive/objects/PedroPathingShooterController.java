package org.firstinspires.ftc.teamcode.drive.objects;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

/**
 * Controlador para o Shooter baseado na distância do alvo usando Pedro Pathing.
 * Ajusta a potência do motor HD Hex (sem redução) proporcionalmente à distância.
 */
public class PedroPathingShooterController {
    private Follower follower;
    public DcMotorEx shooterMotor;

    // Coordenadas do Alvo (Poste/Cesta) no Campo (em polegadas)
    private double targetX = 138;
    private double targetY = 138;

    // Limites de Potência
    private double minPower = 0.4;
    private double maxPower = 0.9;
    // Distâncias de Referência (em polegadas)
    // Ajuste conforme os testes na arena
    private double minDistance = 20.0; // Distância onde a potência é mínima (0.35)
    private double maxDistance = 120.0; // Distância onde a potência é máxima (0.90)

    public void init(HardwareMap hardwareMap, Follower follower, String motorName) {
        this.follower = follower;
        this.shooterMotor = hardwareMap.get(DcMotorEx.class, motorName);

        // Shooter geralmente usa RUN_USING_ENCODER para manter velocidade constante
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * Define a posição do alvo para o cálculo de distância.
     */
    public void setTargetPosition(double x, double y) {
        this.targetX = x;
        this.targetY = y;
    }

    /**
     * Define os limites de potência e distâncias de referência.
     */
    public void setPowerConfig(double minP, double maxP, double minD, double maxD) {
        this.minPower = minP;
        this.maxPower = maxP;
        this.minDistance = minD;
        this.maxDistance = maxD;
    }

    public void update() {
        Pose currentPose = follower.getPose();

        // 1. Calcular a distância euclidiana até o alvo
        double dx = targetX + currentPose.getX();
        double dy = targetY + currentPose.getY();
//        double dx = currentPose.getX() - targetX;
//        double dy = currentPose.getY() - targetY;
        double distance = Math.sqrt(dx * dx + dy * dy);

        // 2. Interpolação Linear da Potência
        // Mapeia a distância [minDistance, maxDistance] para a potência [minPower, maxPower]
        double power;
        if (distance <= minDistance) {
            power = minPower;
        } else if (distance >= maxDistance) {
            power = maxPower;
        } else {
            // Cálculo da interpolação: y = y1 + (x - x1) * (y2 - y1) / (x2 - x1)
            power = minPower + (distance - minDistance) * (maxPower - minPower) / (maxDistance - minDistance);
        }

        // 3. Aplicar potência ao motor
        shooterMotor.setPower(power);
    }

    public double getDistance() {
        Pose currentPose = follower.getPose();
        double dx = targetX - currentPose.getX();
        double dy = targetY - currentPose.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }

    public double getCurrentPower() {
        return shooterMotor.getPower();
    }

    public void stop() {
        shooterMotor.setPower(0);
    }
}
