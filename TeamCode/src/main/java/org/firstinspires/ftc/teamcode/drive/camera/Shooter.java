package org.firstinspires.ftc.teamcode.drive.camera;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Shooter com Limelight", group="Vision")
public class Shooter extends LinearOpMode {

    private Limelight3A limelight;
    private org.firstinspires.ftc.teamcode.drive.objects.Shooter shooter;
    private DistanceSensor sensorDistance;
    private DcMotor Motor;

    // --- PARÂMETROS DE CALIBRAÇÃO (MUDAR CONFORME SEU ROBÔ) ---
    private static final double CAMERA_HEIGHT_M = 0.33; // Altura da câmera em metros
    private static final double CAMERA_ANGLE_DEG = 10.0; // Ângulo de inclinação da câmera em graus
    private static final double TARGET_HEIGHT_M = 0.75; // Altura do centro do AprilTag em metros

    // --- PARÂMETROS DO SHOOTER ---
    private static final double MIN_POWER = 0.3; // Potência mínima do shooter
    private static final double MAX_POWER = 1.0; // Potência máxima do shooter
    private static final double MIN_DISTANCE_M = 0.5; // Distância mínima em metros (ajuste conforme necessário)
    private static final double MAX_DISTANCE_M = 3.0; // Distância máxima em metros (ajuste conforme necessário)
    private static final double POWER_SLOPE = (MAX_POWER - MIN_POWER) / (MAX_DISTANCE_M - MIN_DISTANCE_M); // Inclinação da curva de potência

    private static final double distancia = 110; // Distância para detecção de bola (mm)

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

    /**
     * Calcula a potência do shooter baseada na distância.
     * Usa uma função linear: quanto maior a distância, maior a potência.
     *
     * @param distanceM Distância em metros até o alvo.
     * @return Potência do shooter entre MIN_POWER e MAX_POWER.
     */
    public double calculateShooterPower(double distanceM) {
        // Limitar a distância entre MIN_DISTANCE_M e MAX_DISTANCE_M
        if (distanceM < MIN_DISTANCE_M) {
            distanceM = MIN_DISTANCE_M;
        } else if (distanceM > MAX_DISTANCE_M) {
            distanceM = MAX_DISTANCE_M;
        }

        // Calcular potência usando função linear: power = MIN_POWER + (distance - MIN_DISTANCE) * POWER_SLOPE
        double power = MIN_POWER + (distanceM - MIN_DISTANCE_M) * POWER_SLOPE;

        // Garantir que está dentro dos limites
        if (power < MIN_POWER) {
            power = MIN_POWER;
        } else if (power > MAX_POWER) {
            power = MAX_POWER;
        }

        return power;
    }

    @Override
    public void runOpMode() {
        telemetry.addLine("Inicializando Shooter com Limelight...");
        telemetry.update();

        // Inicialização do hardware
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(7);

        shooter = new org.firstinspires.ftc.teamcode.drive.objects.Shooter(hardwareMap);

        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");
        Motor = hardwareMap.get(DcMotor.class, "index");

        // Cast para Rev2mDistanceSensor se necessário
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistance;

        telemetry.addLine("Hardware inicializado. Pressione Start para continuar.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Controle do index (detecção de bola)
            boolean isDetected = false;
            if(sensorDistance.getDistance(DistanceUnit.MM) < distancia){
                isDetected = true;
                Motor.setPower(0);
            } else {
                Motor.setPower(0.5);
            }
            if (gamepad1.a){
                Motor.setPower(1);
            }

            // Obter dados da Limelight
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                double tyDeg = result.getTy();
                double txDeg = result.getTx();

                // Calcular a distância usando a lógica geométrica
                double distanceM = calculateDistance(
                        TARGET_HEIGHT_M,
                        CAMERA_HEIGHT_M,
                        CAMERA_ANGLE_DEG,
                        tyDeg
                );

                // Calcular a potência do shooter baseada na distância
                double shooterPower = calculateShooterPower(distanceM);

                // Aplicar potência no shooter
                shooter.Shoot(shooterPower);

                // Telemetria
                telemetry.addData("Status", "AprilTag Detectado");
                telemetry.addData("Ângulo Vertical (ty)", "%.2f graus", tyDeg);
                telemetry.addData("Ângulo Horizontal (tx)", "%.2f graus", txDeg);
                telemetry.addData("Distância Horizontal", "%.2f metros (%.0f cm)", distanceM, distanceM * 100.0);
                telemetry.addData("Potência do Shooter", "%.3f", shooterPower);
                telemetry.addData("Bola detectada", isDetected);
                telemetry.addData("Distância sensor", "%.01f mm", sensorDistance.getDistance(DistanceUnit.MM));

            } else {
                // Sem AprilTag visível, parar o shooter
                shooter.Shoot(0);
                telemetry.addLine("Sem AprilTag ou dados inválidos");
                telemetry.addData("Bola detectada", isDetected);
                telemetry.addData("Distância sensor", "%.01f mm", sensorDistance.getDistance(DistanceUnit.MM));
            }

            telemetry.update();
            sleep(50); // Pequena pausa para não sobrecarregar o loop
        }

        // Garantir que a Limelight e o shooter parem ao sair do loop
        limelight.stop();
        shooter.Shoot(0);
    }
}