package org.firstinspires.ftc.teamcode.Davi.drive.unused;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

/**
 * Classe para controlar um servo com base na cor detectada por um sensor de cor.
 *
 * Lógica de controle:
 * - Cor Roxa: Servo para a direita (posição alta).
 * - Cor Verde: Servo para a esquerda (posição baixa).
 * - Nenhuma/Outra Cor: Servo para o centro (posição média).
 */
public class ColorControlledServo {

    // --- ENUMS E CONSTANTES ---

    public enum DetectedColor {
        NONE,
        GREEN,
        PURPLE
    }

    // Nomes de configuração (devem ser ajustados no arquivo de hardware)
    private static final String COLOR_SENSOR_NAME = "sensor";
    private static final String SERVO_NAME = "servo";

    // Limiares de cor (devem ser ajustados com base em testes)
    // Usaremos o Hue (matiz) para diferenciar Verde e Roxo.
    // O Hue é um valor de 0 a 360.
    private static final float HUE_GREEN_MIN = 100; // Exemplo de faixa de Hue para Verde
    private static final float HUE_GREEN_MAX = 150;
    private static final float HUE_PURPLE_MIN = 250; // Exemplo de faixa de Hue para Roxo
    private static final float HUE_PURPLE_MAX = 300;
    private static final float COLOR_SATURATION_THRESHOLD = 0.3f; // Para ignorar cores pálidas/brancas

    // Posições do Servo
    private final double SERVO_RIGHT_POSITION = 0.8; // Posição alta (Roxo)
    private final double SERVO_LEFT_POSITION = 0.2;  // Posição baixa (Verde)
    private final double SERVO_CENTER_POSITION = 0.5; // Posição central (Nenhuma/Outra)

    // --- HARDWARE ---

    private NormalizedColorSensor colorSensor;
    private Servo servo;

    // --- ESTADO ---

    private DetectedColor lastDetectedColor = DetectedColor.NONE;

    /**
     * Inicializa o controlador.
     * @param hardwareMap O mapa de hardware do OpMode.
     */
    public void init(HardwareMap hardwareMap) {
        // Inicializa Sensor de Cor
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, COLOR_SENSOR_NAME);

        // Inicializa Servo
        servo = hardwareMap.get(Servo.class, SERVO_NAME);

        // Define a posição inicial (centro)
        servo.setPosition(SERVO_CENTER_POSITION);
    }

    /**
     * Determina a cor da bola a partir do sensor.
     * @return A cor detectada.
     */
    private DetectedColor determineColor() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        float[] hsvValues = new float[3];
        android.graphics.Color.colorToHSV(colors.toColor(), hsvValues);

        float hue = hsvValues[0];
        float saturation = hsvValues[1];

        if (saturation < COLOR_SATURATION_THRESHOLD) {
            return DetectedColor.NONE; // Cor muito pálida/branca
        }

        // Verifica a cor Verde
        if (hue >= HUE_GREEN_MIN && hue <= HUE_GREEN_MAX) {
            return DetectedColor.GREEN;
        }

        // Verifica a cor Roxa (Purple)
        if (hue >= HUE_PURPLE_MIN && hue <= HUE_PURPLE_MAX) {
            return DetectedColor.PURPLE;
        }

        return DetectedColor.NONE;
    }

    /**
     * Atualiza a posição do servo com base na cor detectada.
     */
    public void update() {
        lastDetectedColor = determineColor();
        double targetPosition = SERVO_CENTER_POSITION;

        switch (lastDetectedColor) {
            case PURPLE:
                targetPosition = SERVO_RIGHT_POSITION;
                break;
            case GREEN:
                targetPosition = SERVO_LEFT_POSITION;
                break;
            case NONE:
            default:
                targetPosition = SERVO_CENTER_POSITION;
                break;
        }

        servo.setPosition(targetPosition);
    }

    // --- MÉTODOS GETTERS PARA TELEMETRIA ---

    public DetectedColor getLastDetectedColor() {
        return lastDetectedColor;
    }

    public double getCurrentServoPosition() {
        return servo.getPosition();
    }
}
