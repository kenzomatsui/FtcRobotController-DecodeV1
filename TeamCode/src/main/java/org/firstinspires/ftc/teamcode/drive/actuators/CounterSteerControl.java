package org.firstinspires.ftc.teamcode.drive.actuators;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Classe para controle de contraesterço baseado no sensor Pinpoint.
 * Implementa controle PID, limites de zona e inversão automática de direção.
 */
@Config
public class CounterSteerControl {
    private DcMotorEx steeringMotor;
    private GoBildaPinpointDriver pinpoint;

    // Parâmetros PID
    public static double kP = 0.05;
    public static double kI = 0.0;
    public static double kD = 0.01;

    private double integralSum = 0;
    private double lastError = 0;
    private ElapsedTime timer = new ElapsedTime();

    // Configurações de Zona
    public static double minLimit = -45.0; // graus
    public static double maxLimit = 45.0;  // graus
    private boolean movingTowardsMax = true;
    private boolean autoReverseEnabled = true;

    // Configurações de Contraesterço
    private double counterSteerGain = 1.0;
    private double lastPinpointHeading = 0;

    // Redução do motor 15:1
    // Ajuste conforme os ticks por revolução do seu motor específico
    private static final double TICKS_PER_REV = 28.0 * 15.0;

    public CounterSteerControl(DcMotorEx motor, GoBildaPinpointDriver pinpoint) {
        this.steeringMotor = motor;
        this.pinpoint = pinpoint;
        this.timer.reset();
    }

    /**
     * Define os ganhos do PID.
     */
    public void setPID(double p, double i, double d) {
        this.kP = p;
        this.kI = i;
        this.kD = d;
    }

    /**
     * Define os limites da zona de atuação em graus.
     */
    public void setLimits(double min, double max) {
        this.minLimit = min;
        this.maxLimit = max;
    }

    /**
     * Define o ganho do contraesterço.
     */
    public void setCounterSteerGain(double gain) {
        this.counterSteerGain = gain;
    }

    /**
     * Atualiza a lógica de controle. Deve ser chamado em cada loop do OpMode.
     */
    public void update() {
        // 1. Obter dados do Pinpoint
        pinpoint.update();
        double currentPinpointHeading = pinpoint.getHeading(AngleUnit.DEGREES);
        double headingDelta = currentPinpointHeading - lastPinpointHeading;
        lastPinpointHeading = currentPinpointHeading;

        // 2. Lógica de Contraesterço
        // Se o robô gira para a direita (positivo), o motor atua para a esquerda (negativo)
        double targetAngle = -headingDelta * counterSteerGain;

        // 3. Lógica de Zona e Inversão Automática
        double currentMotorAngle = getMotorAngle();

        if (autoReverseEnabled) {
            if (movingTowardsMax) {
                targetAngle = maxLimit;
                if (currentMotorAngle >= maxLimit - 2) { // Tolerância de 2 graus
                    movingTowardsMax = false;
                }
            } else {
                targetAngle = minLimit;
                if (currentMotorAngle <= minLimit + 2) {
                    movingTowardsMax = true;
                }
            }
        } else {
            // Se não estiver em modo auto-reverse, apenas limita o target
            targetAngle = Range.clip(targetAngle, minLimit, maxLimit);
        }

        // 4. Controle PID
        double power = calculatePID(targetAngle, currentMotorAngle);
        steeringMotor.setPower(power);
    }

    /**
     * Calcula a saída do PID.
     */
    private double calculatePID(double target, double current) {
        double error = target - current;
        double deltaTime = timer.seconds();

        if (deltaTime > 0) {
            integralSum += error * deltaTime;
            double derivative = (error - lastError) / deltaTime;
            lastError = error;
            timer.reset();

            double output = (kP * error) + (kI * integralSum) + (kD * derivative);
            return Range.clip(output, -1.0, 1.0);
        }
        return 0;
    }

    /**
     * Converte ticks do encoder para graus considerando a redução 15:1.
     */
    public double getMotorAngle() {
        return (steeringMotor.getCurrentPosition() / TICKS_PER_REV) * 360.0;
    }

    public void setAutoReverse(boolean enabled) {
        this.autoReverseEnabled = enabled;
    }

    public void resetEncoder() {
        steeringMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        steeringMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }
}
