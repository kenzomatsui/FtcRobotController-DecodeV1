package org.firstinspires.ftc.teamcode.drive.camera;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp
public class PIDTa extends LinearOpMode {

    private Limelight3A limelight;
    private DcMotor driveMotor = null; // Motor que vai acelerar conforme a distância

    // Parâmetros ajustáveis no Dashboard
    public static double MIN_POWER = 0.35;   // Potência mínima
    public static double MAX_POWER = 1;   // Potência máxima
    public static double TARGET_TA = 5.0;   // "Área" esperada quando estiver na distância ideal
    public static double SCALE_FACTOR = 0.084; // Ajuste da curva de resposta

    @Override
    public void runOpMode() {
        // Inicializa Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(7);

        // Inicializa o motor
        driveMotor = hardwareMap.get(DcMotor.class, "RMTa"); // Ajuste o nome
        driveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveMotor.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Inicializado");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                double ta = result.getTa(); // Área da AprilTag (relacionada à distância)

                // Calcula o erro relativo à área desejada
                double distanceError = TARGET_TA - ta;

                // Quanto menor a área (mais longe), maior o erro => mais potência
                double power = MIN_POWER + (distanceError * SCALE_FACTOR);

                // Limita entre os valores mínimos e máximos
                power = Math.max(MIN_POWER, Math.min(power, MAX_POWER));

                // Se já estiver muito perto (ta >= TARGET_TA), para o motor
                if (ta >= TARGET_TA) {
                    driveMotor.setPower(0);
                    telemetry.addData("Status", "Distância ideal ou muito perto");
                } else {
                    driveMotor.setPower(power);
                    telemetry.addData("Status", "Aproximando...");
                }

                telemetry.addData("Área (ta)", ta);
                telemetry.addData("Erro de distância", distanceError);
                telemetry.addData("Potência aplicada", power);
            } else {
                // Nenhuma tag visível
                driveMotor.setPower(0);
                telemetry.addData("Status", "Nenhuma AprilTag detectada");
            }

            telemetry.update();
        }
    }
}
