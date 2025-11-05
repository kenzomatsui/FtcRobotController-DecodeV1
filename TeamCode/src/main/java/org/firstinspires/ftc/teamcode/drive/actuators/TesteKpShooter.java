package org.firstinspires.ftc.teamcode.drive.actuators;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name="Limelight3A AprilTag Distancia", group="Vision")
public class TesteKpShooter extends LinearOpMode {

    private Limelight3A limelight;
    private IMU imu;
    private org.firstinspires.ftc.teamcode.drive.objects.Shooter shooter;

    // --- PARÂMETROS DE CALIBRAÇÃO (MUDAR CONFORME SEU ROBÔ) ---
    private static final double CAMERA_HEIGHT_M = 0.33; // Altura da câmera em metros
    private static final double CAMERA_ANGLE_DEG = 10.0; // Ângulo de inclinação da câmera em graus
    private static final double TARGET_HEIGHT_M = 0.75; // Altura do centro do AprilTag em metros
    
    // --- COORDENADAS DO GOAL (TUNAR BASEADO NO GAME MANUAL) ---
    // Coordenadas do centro do goal onde vocês querem atirar (em metros, no sistema de coordenadas do campo)
    // Para aliança AZUL (canto superior esquerdo), ajuste conforme necessário
    private static final double GOAL_X_M = 0.0; // Ajuste com a coordenada X do goal (em metros)
    private static final double GOAL_Y_M = 0.0; // Ajuste com a coordenada Y do goal (em metros)
    // Para aliança VERMELHA, inverta ou use coordenadas diferentes
    // ---------------------------------------------------------

    // --- PARÂMETROS DE POTÊNCIA DO SHOOTER (TUNAR EM CAMPO) ---
    private static final double MIN_POWER = 0.30;       // potência mínima segura
    private static final double MAX_POWER = 1.00;       // potência máxima
    private static final double MIN_DISTANCE_M = 0.50;  // distância mínima considerada
    private static final double MAX_DISTANCE_M = 3.00;  // distância máxima considerada
    private static final double POWER_SLOPE = (MAX_POWER - MIN_POWER) / (MAX_DISTANCE_M - MIN_DISTANCE_M);

    /**
     * Calcula a distância euclidiana (2D) do robô até o goal usando coordenadas X e Y do campo.
     * Isso considera tanto a distância horizontal (X) quanto vertical (Y) do campo.
     * 
     * @param robotX Posição X do robô no campo (em metros)
     * @param robotY Posição Y do robô no campo (em metros)
     * @param goalX Posição X do goal no campo (em metros)
     * @param goalY Posição Y do goal no campo (em metros)
     * @return Distância euclidiana em metros: sqrt((X_goal - X_robot)² + (Y_goal - Y_robot)²)
     */
    public double calculateDistanceXY(double robotX, double robotY, double goalX, double goalY) {
        double deltaX = goalX - robotX;
        double deltaY = goalY - robotY;
        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    /**
     * Método alternativo: calcula distância usando apenas ty (mantido para fallback)
     * Usa trigonometria baseada no ângulo vertical da AprilTag.
     */
    public double calculateDistanceFromTy(double targetHeightM, double cameraHeightM, double cameraAngleDeg, double tyDeg) {
        double cameraAngleRad = Math.toRadians(cameraAngleDeg);
        double tyRad = Math.toRadians(tyDeg);
        double totalAngleRad = cameraAngleRad + tyRad;
        double heightDifference = targetHeightM - cameraHeightM;

        if (Math.abs(totalAngleRad) < 0.001) {
            return Double.MAX_VALUE;
        }

        double distanceM = heightDifference / Math.tan(totalAngleRad);
        return Math.abs(distanceM);
    }

    /**
     * Calcula potência do shooter a partir da distância usando uma curva linear simples.
     * Ajuste MIN_/MAX_ conforme testes de campo.
     */
    public double calculateShooterPower(double distanceM) {
        double d = distanceM;
        if (d < MIN_DISTANCE_M) d = MIN_DISTANCE_M;
        if (d > MAX_DISTANCE_M) d = MAX_DISTANCE_M;
        double power = MIN_POWER + (d - MIN_DISTANCE_M) * POWER_SLOPE;
        if (power < MIN_POWER) power = MIN_POWER;
        if (power > MAX_POWER) power = MAX_POWER;
        return power;
    }

    @Override
    public void runOpMode() {
        telemetry.addLine("Inicializando Limelight3A para cálculo de distância...");
        telemetry.update();

        // Inicialização do hardware
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(7);

        // Inicializar IMU (necessário para BotPose_MT2)
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters myIMUparameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
            )
        );
        imu.initialize(myIMUparameters);

        // Shooter (motores motora/motorb)
        shooter = new org.firstinspires.ftc.teamcode.drive.objects.Shooter(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            // Atualizar orientação do robô para o Limelight (necessário para BotPose_MT2)
            double robotYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            limelight.updateRobotOrientation(robotYaw);

            LLResult result = limelight.getLatestResult();
            double distanceM = 0.0;
            boolean useXY = false;

            if (result != null && result.isValid()) {
                // Tentar obter posição do robô usando BotPose_MT2 (método preferido)
                Pose3D botpose_mt2 = result.getBotpose_MT2();
                
                if (botpose_mt2 != null) {
                    double robotX = botpose_mt2.getPosition().x;
                    double robotY = botpose_mt2.getPosition().y;
                    
                    // Calcular distância usando coordenadas X e Y (método correto para considerar X e Y)
                    distanceM = calculateDistanceXY(robotX, robotY, GOAL_X_M, GOAL_Y_M);
                    useXY = true;
                    
                    // Telemetria
                    telemetry.addData("Status", "AprilTag Detectado (BotPose)");
                    telemetry.addData("Posição Robô", "(%.2f, %.2f) m", robotX, robotY);
                    telemetry.addData("Posição Goal", "(%.2f, %.2f) m", GOAL_X_M, GOAL_Y_M);
                    telemetry.addData("Distância (X,Y)", "%.2f metros (%.0f cm)", distanceM, distanceM * 100.0);
                    
                } else {
                    // Fallback: usar método ty se BotPose não estiver disponível
                    double tyDeg = result.getTy();
                    distanceM = calculateDistanceFromTy(
                        TARGET_HEIGHT_M,
                        CAMERA_HEIGHT_M,
                        CAMERA_ANGLE_DEG,
                        tyDeg
                    );
                    
                    telemetry.addData("Status", "AprilTag Detectado (Fallback ty)");
                    telemetry.addData("Ângulo Vertical (ty)", "%.2f graus", tyDeg);
                    telemetry.addData("Distância (ty)", "%.2f metros (%.0f cm)", distanceM, distanceM * 100.0);
                }

                // Calcular potência do shooter e aplicar
                double shooterPower = calculateShooterPower(distanceM);
                shooter.Shoot(shooterPower);

                telemetry.addData("Método", useXY ? "Coordenadas X,Y" : "Fallback ty");
                telemetry.addData("Potência Shooter", "%.3f", shooterPower);
                telemetry.addData("Ângulo Horizontal (tx)", "%.2f graus", result.getTx());
                telemetry.addData("Yaw do Robô", "%.2f graus", robotYaw);

            } else {
                telemetry.addLine("Sem AprilTag ou dados inválidos");
                shooter.Shoot(0);
            }

            telemetry.update();
            sleep(50); // Pequena pausa para não sobrecarregar o loop
        }

        // Garantir que a Limelight e o shooter parem ao sair do loop
        limelight.stop();
        shooter.Shoot(0);
    }
}
