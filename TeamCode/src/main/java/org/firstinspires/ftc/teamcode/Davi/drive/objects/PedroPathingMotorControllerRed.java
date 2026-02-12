package org.firstinspires.ftc.teamcode.Davi.drive.objects;

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
 * Suporta limites estendidos (ex: -60 a 300 graus).
 */
public class PedroPathingMotorControllerRed {
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
    private double minLimit = -60.0;
    private double maxLimit = 260.0;

    // Coordenadas do Alvo (Poste) no Campo (Pedro Pathing usa polegadas)
    private double targetX = 138;
    private double targetY = 138;

    private boolean isLocked = false;
    private double lockedAngle = 0;

    // Redução Total 15:1 (28 ticks * 3 * 5 = 420)
    private static final double TICKS_PER_REV = 2100.0;

    public void init(HardwareMap hardwareMap, Follower follower, String motorName) {
        this.follower = follower;
        this.motor = hardwareMap.get(DcMotorEx.class, motorName);

        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        timer.reset();
    }

    public void setTargetPosition(double x, double y) {
        this.targetX = x;
        this.targetY = y;
    }
    public void lockAngle(double angle) {
        this.lockedAngle = angle;
        this.isLocked = true;
    }

    /**
     * Libera a turret da trava de ângulo, voltando ao rastreio automático.
     */
    public void unlockAngle() {
        this.isLocked = false;
    }

    public void update() {
        double currentMotorAngle = getMotorAngle();
        double targetDegrees;

        if (isLocked) {
            // Se estiver travado, o alvo é o ângulo definido
            targetDegrees = lockedAngle;
        } else {
            Pose currentPose = follower.getPose();


            // 1. Calcular o ângulo absoluto do alvo em relação ao campo
            double dx = targetX + currentPose.getX();
            double dy = targetY + currentPose.getY();
            double absoluteAngleToTarget = Math.toDegrees(Math.atan2(dy, dx));

            // 2. Calcular o ângulo relativo ao robô
            double robotHeading = Math.toDegrees(currentPose.getHeading());
            double relativeTargetAngle = absoluteAngleToTarget - robotHeading;

            // Normaliza o erro para o intervalo [-180, 180] para encontrar o caminho mais curto
            double error = relativeTargetAngle - (currentMotorAngle % 360);
            while (error > 180) error -= 360;
            while (error < -180) error += 360;

            // O alvo real é a posição atual + o caminho mais curto
            targetDegrees = currentMotorAngle + error;
        }

        // 4. Lógica de Zona: Travar nos limites
        // Agora o clip funciona corretamente mesmo para 300 graus
        double constrainedTargetDegrees = Range.clip(targetDegrees, minLimit, maxLimit);

        // 5. Controle PID
        double power = calculatePID(constrainedTargetDegrees, currentMotorAngle);
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

    public double getMotorAngle() {
        // Retorna a posição real acumulada do motor em graus
        return (motor.getCurrentPosition() / TICKS_PER_REV) * 360.0;
    }

    public void setLimits(double min, double max) {
        this.minLimit = min;
        this.maxLimit = max;
    }
}
