package org.firstinspires.ftc.teamcode.drive.actuators;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Controlador para motor em contraesterço baseado no sensor Pinpoint.
 * Corrigido para priorizar a reação ao movimento do Pinpoint.
 */
public class PinpointMotorController {
    private GoBildaPinpointDriver pinpoint;
    private DcMotorEx motor;

    // Parâmetros PID para motor 15:1
    private double kP = 0.01;
    private double kI = 0.0;
    private double kD = 0.001;

    private double integralSum = 0;
    private double lastError = 0;
    private ElapsedTime timer = new ElapsedTime();

    // Configurações de Zona e Movimento
    private double minLimit = -60.0; // graus
    private double maxLimit = 60.0;  // graus
    private boolean movingTowardsMax = true;
    private boolean autoReverseEnabled = true;

    private double counterSteerGain = 1.5; // Ajuste para sensibilidade do contraesterço
    private double currentHeading = 0;

    // Redução 15:1
    private static final double TICKS_PER_REV = 28.0 * 15.0;

    public void init(HardwareMap hardwareMap, String pinpointName, String motorName) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, pinpointName);
        motor = hardwareMap.get(DcMotorEx.class, motorName);

        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        timer.reset();
    }

    public void update() {
        pinpoint.update();
        currentHeading = pinpoint.getHeading(AngleUnit.DEGREES);

        double currentMotorAngle = getMotorAngle();
        double targetAngle;

        // 1. Cálculo da base do movimento (Auto-Reverse)
        double baseTarget;
        if (autoReverseEnabled) {
            if (movingTowardsMax) {
                baseTarget = maxLimit;
                if (currentMotorAngle >= maxLimit - 5) movingTowardsMax = false;
            } else {
                baseTarget = minLimit;
                if (currentMotorAngle <= minLimit + 5) movingTowardsMax = true;
            }
        } else {
            baseTarget = 0; // Mantém no centro se auto-reverse desligado
        }

        // 2. Aplicação do Contraesterço
        // O contraesterço é somado ao movimento base.
        // Se o robô gira para a direita (+), o motor compensa para a esquerda (-)
        targetAngle = baseTarget - (currentHeading * counterSteerGain);

        // 3. Limitação Final (Segurança Física)
        targetAngle = Range.clip(targetAngle, minLimit - 10, maxLimit + 10);

        // 4. Controle PID
        double power = calculatePID(targetAngle, currentMotorAngle);
        motor.setPower(power);
    }

    private double calculatePID(double target, double current) {
        double error = target - current;
        double deltaTime = timer.seconds();

        if (deltaTime > 0) {
            integralSum += error * deltaTime;
            // Limita integral para evitar windup
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
        return (motor.getCurrentPosition() / TICKS_PER_REV) * 360.0;
    }

    public double getCurrentHeading() {
        return currentHeading;
    }

    public double getMotorPower() {
        return motor.getPower();
    }

    public void setCounterSteerGain(double gain) {
        this.counterSteerGain = gain;
    }

    public void setLimits(double min, double max) {
        this.minLimit = min;
        this.maxLimit = max;
    }

    public void setAutoReverse(boolean enabled) {
        this.autoReverseEnabled = enabled;
    }
}
