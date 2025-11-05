package org.firstinspires.ftc.teamcode.drive.actuators;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Limelight3A AprilTag Distancia", group="Vision")
public class TesteKpShooter extends LinearOpMode {

    private Limelight3A limelight;

    // --- PARÂMETROS DE CALIBRAÇÃO (MUDAR CONFORME SEU ROBÔ) ---
    private static final double CAMERA_HEIGHT_M = 0.33; // Exemplo: 15 cm
    private static final double CAMERA_ANGLE_DEG = 10.0; // Exemplo: 15 graus para cima
    private static final double TARGET_HEIGHT_M = 0.75; // Exemplo: 80 cm (altura do centro do AprilTag)
    // ---------------------------------------------------------

    /**
     * Calcula a distância horizontal até um alvo (AprilTag) usando a trigonometria
     * de um triângulo retângulo, com base no ângulo vertical do alvo (ty) e
     * nas dimensões de montagem da câmera.
     *
     * @param targetHeightM Altura do centro do AprilTag em metros.
     * @param cameraHeightM Altura do centro da lente da Limelight em metros.
     * @param cameraAngleDeg Ângulo de inclinação (pitch) da Limelight em graus, em relação ao chão.
     * @param tyDeg O ângulo vertical do alvo em relação ao centro da Limelight (valor 'ty').
     * @return A distância horizontal em metros.
     */
    public double calculateDistance(double targetHeightM, double cameraHeightM, double cameraAngleDeg, double tyDeg) {
        // 1. Converter todos os ângulos para radianos para uso com Math.tan
        double cameraAngleRad = Math.toRadians(cameraAngleDeg);
        double tyRad = Math.toRadians(tyDeg);

        // 2. Calcular o ângulo total do alvo em relação ao chão
        double totalAngleRad = cameraAngleRad + tyRad;

        // 3. Aplicar a fórmula trigonométrica: Distância = (Diferença de Altura) / tan(Ângulo Total)
        double heightDifference = targetHeightM - cameraHeightM;

        // Evitar divisão por zero
        if (Math.abs(totalAngleRad) < 0.001) {
            return Double.MAX_VALUE;
        }

        double distanceM = heightDifference / Math.tan(totalAngleRad);

        // A distância horizontal deve ser positiva
        return Math.abs(distanceM);
    }

    @Override
    public void runOpMode() {
        telemetry.addLine("Inicializando Limelight3A para cálculo de distância...");
        telemetry.update();

        // Inicialização do hardware
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // Set how often we ask Limelight for data (100 times per second)
        limelight.start(); // Tell Limelight to start looking!
        limelight.pipelineSwitch(7);

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                double tyDeg = result.getTy();

                // Calcular a distância usando a lógica geométrica
                double distanceM = calculateDistance(
                        TARGET_HEIGHT_M,
                        CAMERA_HEIGHT_M,
                        CAMERA_ANGLE_DEG,
                        tyDeg
                );

                // *** LINHA DE TELEMETRIA SOLICITADA ***
                telemetry.addData("Status", "AprilTag Detectado");
                telemetry.addData("Ângulo Vertical (ty)", "%.2f graus", tyDeg);
                telemetry.addData("Distância Horizontal", "%.2f metros (%.0f cm)", distanceM, distanceM * 100.0);
                // Opcional: Mostrar o ângulo horizontal para alinhamento
                telemetry.addData("Ângulo Horizontal (tx)", "%.2f graus", result.getTx());

            } else {
                telemetry.addLine("Sem AprilTag ou dados inválidos");
            }

            telemetry.update();
            sleep(50); // Pequena pausa para não sobrecarregar o loop
        }

        // Garantir que a Limelight pare ao sair do loop
        limelight.stop();
    }
}
