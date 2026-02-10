package org.firstinspires.ftc.teamcode.drive.objects;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Classe para controlar dois servos em contraesterço (counter-steer)
 * com base no ângulo de guinada (heading) do sensor GoBilda Pinpoint.
 *
 * O objetivo é estabilizar ou compensar a rotação do robô, movendo os servos
 * em direções opostas (contraesterço) proporcionalmente ao erro de ângulo.
 */
public class PinpointServoController {

    private Servo servoLeft;
    private Servo servoRight;
    private GoBildaPinpointDriver pinpoint;

    // Posições neutras dos servos (devem ser ajustadas para o seu robô)
    private final double CENTER_POSITION_LEFT = 0.5;
    private final double CENTER_POSITION_RIGHT = 0.5;

    // Máxima deflexão do servo (e.g., 0.25 significa que o servo irá de 0.25 a 0.75)
    private final double MAX_DEFLECTION = 0.5;

    // Ângulo máximo em graus para atingir a deflexão máxima do servo
    private final double MAX_ANGLE_DEGREES = 270.0;

    /**
     * Inicializa o controlador de servos.
     * @param hardwareMap O mapa de hardware do OpMode.
     * @param pinpointName O nome de configuração do sensor Pinpoint.
     * @param servoLeftName O nome de configuração do servo esquerdo.
     * @param servoRightName O nome de configuração do servo direito.
     */
    public void init(HardwareMap hardwareMap, String pinpointName, String servoLeftName, String servoRightName) {
        // Inicializa o Pinpoint
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, pinpointName);

        // Inicializa os Servos
        servoLeft = hardwareMap.get(Servo.class, servoLeftName);
        servoRight = hardwareMap.get(Servo.class, servoRightName);

        // Configura a direção do servo direito para que 0.0 e 1.0 sejam invertidos
        // Isso é crucial para o movimento de "contraesterço"
        servoRight.setDirection(Servo.Direction.REVERSE);

        // Define a posição inicial (centro)
        servoLeft.setPosition(CENTER_POSITION_LEFT);
        servoRight.setPosition(CENTER_POSITION_RIGHT);
    }

    /**
     * Atualiza a posição dos servos com base no ângulo de guinada (heading) do Pinpoint.
     * Assume que o robô está tentando manter um heading de 0 graus.
     */
    public void update() {
        // 1. Atualiza e obtém a pose do Pinpoint
        pinpoint.update();
        Pose2D pose2D = pinpoint.getPosition();

        // 2. Obtém o ângulo de guinada (heading) em graus. O Pinpoint retorna entre -180 e 180.
        double currentHeading = pose2D.getHeading(AngleUnit.DEGREES);

        // 3. Calcula o erro de ângulo (assumindo que o alvo é 0 graus)
        double angleError = currentHeading;

        // 4. Limita o erro de ângulo ao range de controle (e.g., -45 a 45 graus)
        // Isso evita que os servos girem descontroladamente para grandes erros.
        double clampedError = Math.max(-MAX_ANGLE_DEGREES, Math.min(MAX_ANGLE_DEGREES, angleError));

        // 5. Calcula a razão de deflexão (entre -1.0 e 1.0)
        double deflectionRatio = clampedError / MAX_ANGLE_DEGREES;

        // 6. Calcula a mudança de posição do servo
        // Se o erro for positivo (robô girou para a direita), deflectionChange será positivo.
        double servoChange = deflectionRatio * MAX_DEFLECTION;

        // 7. Aplica a lógica de contraesterço
        // Servo Esquerdo: Adiciona a mudança. Se o robô girar para a direita (erro positivo),
        // o servo esquerdo se move para a posição mais alta (e.g., 0.5 + 0.25 = 0.75).
        double targetPositionLeft = CENTER_POSITION_LEFT + servoChange;

        // Servo Direito: Subtrai a mudança. Se o robô girar para a direita (erro positivo),
        // o servo direito se move para a posição mais baixa (e.g., 0.5 - 0.25 = 0.25).
        // Como a direção do servo direito foi invertida na inicialização,
        // a posição mais baixa (0.25) na verdade o move na direção oposta ao servo esquerdo.
        double targetPositionRight = CENTER_POSITION_RIGHT - servoChange;

        // 8. Garante que as posições estejam dentro do limite [0.0, 1.0]
        targetPositionLeft = Math.max(0.0, Math.min(1.0, targetPositionLeft));
        targetPositionRight = Math.max(0.0, Math.min(1.0, targetPositionRight));

        // 9. Define as novas posições dos servos
        servoLeft.setPosition(targetPositionLeft);
        servoRight.setPosition(targetPositionRight);
    }

    /**
     * Retorna a posição atual do servo esquerdo.
     */
    public double getLeftPosition() {
        return servoLeft.getPosition();
    }

    /**
     * Retorna a posição atual do servo direito.
     */
    public double getRightPosition() {
        return servoRight.getPosition();
    }

    /**
     * Retorna o heading atual do Pinpoint em graus.
     */
    public double getCurrentHeading() {
        pinpoint.update();
        return pinpoint.getPosition().getHeading(AngleUnit.DEGREES);
    }
}
