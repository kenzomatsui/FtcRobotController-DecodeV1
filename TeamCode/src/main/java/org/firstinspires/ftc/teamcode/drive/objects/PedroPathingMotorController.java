package org.firstinspires.ftc.teamcode.drive.objects;

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
 * Lógica de limites: O motor para no limite em vez de inverter o movimento.
 */
public class PedroPathingMotorController {
    private Follower follower;
    private DcMotorEx motor;

    // Parâmetros PID para motor 15:1
    private double kP = 0.06; //assim ta bom
    private double kI = 0.0;
    private double kD = 0.0005;

    private double integralSum = 0;
    private double lastError = 0;
    private ElapsedTime timer = new ElapsedTime();

    // Configurações de Zona
    private double minLimit = 0;
    private double maxLimit = 180.0;

    // Coordenadas do Alvo (Poste) no Campo (Pedro Pathing usa polegadas)
    private double targetX = 0;
    private double targetY = 116;

    // Redução Total 15:1 (28 ticks * 3 * 5 = 420)
    private static final double TICKS_PER_REV = 420.0;

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

    public void update() {
        // Pega a posição atual do robô do Pedro Pathing
        Pose currentPose = follower.getPose();

        // 1. Recalcular dx e dy a cada frame para garantir correção de translação
        double dx = targetX + currentPose.getX();
        double dy = targetY + currentPose.getY();

        // Ângulo absoluto do alvo em relação ao campo (Radianos)
        double absoluteAngleToTarget = Math.atan2(dy, dx);

        // Ângulo relativo ao robô (Robot Centric)
        double robotHeading = currentPose.getHeading();
        double relativeTargetAngle = absoluteAngleToTarget - robotHeading;

        // Normalizar o ângulo [-PI, PI]
        relativeTargetAngle = normalizeAngle(relativeTargetAngle);

        // Converter para graus para a lógica de zona
        double targetDegrees = Math.toDegrees(relativeTargetAngle);

        // 2. Lógica de Zona: Travar nos limites
        // Em vez de inverter, apenas limitamos o alvo para que ele nunca ultrapasse os limites físicos.
        double constrainedTargetDegrees = Range.clip(targetDegrees, minLimit, maxLimit);

        // 3. Controle PID
        double currentMotorAngle = getMotorAngle();
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
}
