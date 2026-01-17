package org.firstinspires.ftc.teamcode.drive.actuators;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Controlador para base giratória (turret) integrado ao Pedro Pathing.
 * Configurado para motor HD Hex (3:1) + Engrenagem externa (5:1) = 15:1 total.
 */
public class PedroPathingMotorController {
    private Follower follower;
    private DcMotorEx motor;

    // Parâmetros PID para motor 15:1
    private double kP = 0.06;
    private double kI = 0.0;
    private double kD = 0.0005;

    private double integralSum = 0;
    private double lastError = 0;
    private ElapsedTime timer = new ElapsedTime();

    // Configurações de Zona
    private double minLimit = -150.0;
    private double maxLimit = 150.0;
    private boolean autoReverseEnabled = true;
    private boolean movingTowardsMax = true;

    // Coordenadas do Alvo (Poste) no Campo (Pedro Pathing usa polegadas por padrão)
    private double targetX = 0; // Exemplo: Centro do campo em Pedro Pathing
    private double targetY = 50;

    // Redução Total 15:1
    // HD Hex (28 ticks/rev no encoder interno) * 3 (UltraPlanetary) * 5 (Engrenagem externa)
    private static final double TICKS_PER_REV = 28.0 * 3.0 * 5.0;

    public void init(HardwareMap hardwareMap, Follower follower, String motorName) {
        this.follower = follower;
        this.motor = hardwareMap.get(DcMotorEx.class, motorName);

        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        timer.reset();
    }

    /**
     * Define a posição do alvo (poste) no campo.
     * @param x Coordenada X (em polegadas, padrão Pedro Pathing)
     * @param y Coordenada Y (em polegadas, padrão Pedro Pathing)
     */
    public void setTargetPosition(double x, double y) {
        this.targetX = x;
        this.targetY = y;
    }

    public void update() {
        // Pedro Pathing atualiza a posição no Follower.update() que deve ser chamado no OpMode
        Pose currentPose = follower.getPose();

        // 1. Calcular o ângulo do robô em relação ao alvo (Field Centric)
        double dx = targetX + currentPose.getX();
        double dy = targetY + currentPose.getY();

        // Ângulo absoluto do alvo em relação ao campo (Pedro Pathing usa radianos)
        double absoluteAngleToTarget = Math.atan2(dy, dx);

        // Ângulo relativo ao robô (Robot Centric)
        double robotHeading = currentPose.getHeading();
        double relativeTargetAngle = absoluteAngleToTarget - robotHeading;

        // Normalizar o ângulo [-PI, PI]
        relativeTargetAngle = normalizeAngle(relativeTargetAngle);

        // Converter para graus para a lógica de zona
        double targetDegrees = Math.toDegrees(relativeTargetAngle);

        // 2. Lógica de Zona e Inversão Automática
        double currentMotorAngle = getMotorAngle();

        if (autoReverseEnabled) {
            if (targetDegrees > maxLimit || targetDegrees < minLimit) {
                if (movingTowardsMax) {
                    targetDegrees = maxLimit;
                    if (currentMotorAngle >= maxLimit - 5) movingTowardsMax = false;
                } else {
                    targetDegrees = minLimit;
                    if (currentMotorAngle <= minLimit + 5) movingTowardsMax = true;
                }
            }
        } else {
            targetDegrees = Range.clip(targetDegrees, minLimit, maxLimit);
        }

        // 3. Controle PID
        double power = calculatePID(targetDegrees, currentMotorAngle);
        motor.setPower(power);
    }

    private double calculatePID(double target, double current) {
        double error = target - current;
        double deltaTime = timer.seconds();

        if (deltaTime > 0) {
            integralSum += error * deltaTime;
            integralSum = Range.clip(integralSum, -0.5, 0.5);
            double derivative = (error - lastError) / deltaTime;
            lastError = error;
            timer.reset();

            double output = (kP * error) + (kI * integralSum) + (kD * derivative);
            return Range.clip(output, -1.0, 1.0);
        }
        return 0;
    }

    private double normalizeAngle(double radians) {
        while (radians > Math.PI) radians -= 2 * Math.PI;
        while (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }

    public double getMotorAngle() {
        return (motor.getCurrentPosition() / TICKS_PER_REV) * 360.0;
    }

    public Pose getCurrentPose() {
        return follower.getPose();
    }

    public double getMotorPower() {
        return motor.getPower();
    }

    public void setLimits(double min, double max) {
        this.minLimit = min;
        this.maxLimit = max;
    }

    public void setAutoReverse(boolean enabled) {
        this.autoReverseEnabled = enabled;
    }
}
