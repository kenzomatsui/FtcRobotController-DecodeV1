package org.firstinspires.ftc.teamcode.Davi.drive.actuators;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

/**
 * Implementação de Localização Estilo MegaTag2 para FTC.
 * Corrigido para acessar o Botpose corretamente na biblioteca Limelight FTC.
 */
public class KalmanFilterLocalizer {
    private Follower follower;
    private Limelight3A limelight;

    // Estado atual fundido [x, y, heading]
    private Pose fusedPose = new Pose(0, 0, 0);

    // Configurações de Confiança (Desvios Padrão)
    private double pinpointStdDev = 0.05;
    private double baseVisionStdDev = 0.1;

    private Pose lastLimelightPose = new Pose(0, 0, 0);
    private int lastTagCount = 0;
    private boolean hasTarget = false;

    public void init(HardwareMap hardwareMap, Follower follower) {
        this.follower = follower;
        this.limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.limelight.pipelineSwitch(0);
        this.limelight.start();

        this.fusedPose = follower.getPose();
    }

    public void update() {
        // 1. Obter orientação atual do Pinpoint para ajudar a Limelight (Lógica MegaTag2)
        Pose currentPinpointPose = follower.getPose();
        double currentHeading = Math.toDegrees(currentPinpointPose.getHeading());

        // Enviar orientação para a Limelight 3A
        limelight.updateRobotOrientation(currentHeading);

        // 2. Obter medição da Limelight
        LLResult result = limelight.getLatestResult();

        // CORREÇÃO: No FTC, usamos getBotpose() que retorna a pose relativa ao centro do campo.
        // A distinção de aliança geralmente é feita na configuração do Field Layout na Limelight.
        if (result != null && result.isValid() && result.getBotpose() != null) {
            Pose3D botpose = result.getBotpose();

            // Contagem de tags visíveis
            lastTagCount = result.getFiducialResults().size();
            if (lastTagCount == 0) lastTagCount = 1;

            // Conversão Metros -> Polegadas
            double visionX = botpose.getPosition().x * 39.3701;
            double visionY = botpose.getPosition().y * 39.3701;
            double visionHeading = Math.toRadians(botpose.getOrientation().getYaw());

            lastLimelightPose = new Pose(visionX, visionY, visionHeading);
            hasTarget = true;

            // 3. Lógica de Média Ponderada
            double currentVisionStdDev = baseVisionStdDev * (1.0 / lastTagCount);

            double wPinpoint = 1.0 / (pinpointStdDev * pinpointStdDev);
            double wVision = 1.0 / (currentVisionStdDev * currentVisionStdDev);
            double totalWeight = wPinpoint + wVision;

            double fusedX = (currentPinpointPose.getX() * wPinpoint + visionX * wVision) / totalWeight;
            double fusedY = (currentPinpointPose.getY() * wPinpoint + visionY * wVision) / totalWeight;

            double angleDiff = visionHeading - currentPinpointPose.getHeading();
            while (angleDiff > Math.PI) angleDiff -= 2 * Math.PI;
            while (angleDiff < -Math.PI) angleDiff += 2 * Math.PI;

            double fusedHeading = currentPinpointPose.getHeading() + (angleDiff * (wVision / totalWeight));

            fusedPose = new Pose(fusedX, fusedY, fusedHeading);

            // 4. Atualizar o Follower com a posição fundida
            follower.setPose(fusedPose);

        } else {
            hasTarget = false;
            fusedPose = currentPinpointPose;
        }
    }

    public Pose getFusedPose() {
        return fusedPose;
    }

    public Pose getLimelightPose() {
        return lastLimelightPose;
    }

    public int getTagCount() {
        return lastTagCount;
    }

    public boolean hasVision() {
        return hasTarget;
    }
}
