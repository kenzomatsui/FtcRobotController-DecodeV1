package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

// Importações do FTC SDK para Limelight
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * OpMode de exemplo para usar a Limelight 3A para detectar uma AprilTag
 * e ajustar a potência de um motor com base na área do alvo (ta),
 * utilizando as classes do FTC SDK (com.qualcomm.hardware.limelightvision).
 *
 * CORREÇÃO: O método getTargetFound() foi corrigido para o método correto
 * para verificar a presença do alvo na classe LLResult do FTC SDK.
 *
 * PRE-REQUISITOS:
 * 1. A Limelight 3A deve estar configurada com um pipeline de AprilTag.
 * 2. O motor deve estar configurado no arquivo de configuração do robô com o nome "motor_distancia".
 * 3. O pipeline da Limelight deve estar ativo.
 */
@Config
@TeleOp(name = "Limelight AprilTag Motor Control (FTC SDK V2)", group = "Limelight")
public class TesteKpShooter extends LinearOpMode {
    public static double distancia = 110;
    private DistanceSensor sensorDistance;
    private DcMotor Motor;

    // Objeto Limelight, usando a classe do FTC SDK
    private Limelight3A limelight;

    // Motor que será controlado pela distância
    private DcMotor motorDistancia;

    // Área Alvo (ta) em porcentagem (0-100).
    // Um valor maior de 'ta' significa que o robô está mais perto.
    // Exemplo: 5.0% da área da imagem. Ajuste este valor experimentalmente.
    private final double TARGET_AREA_PERCENT = 5.0;

    // Constante de ganho para o controle P (Proporcional). Ajuste conforme a necessidade.
    public static double KP = 0.15;

    @Override
    public void runOpMode() {
        // 1. Inicialização do Hardware
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // Isso define com que frequência pedimos dados ao Limelight (100 vezes por segundo)
        limelight.start(); // Isso diz ao Limelight para começar a procurar!
        limelight.pipelineSwitch(7); // Mudar para o pipeline número 7
        motorDistancia = hardwareMap.get(DcMotor.class, "RMTa");

        // Configuração do motor (ajuste a direção conforme a montagem)
        motorDistancia.setDirection(DcMotorSimple.Direction.FORWARD);
        motorDistancia.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // you can use this as a regular DistanceSensor.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");
        Motor = hardwareMap.get(DcMotor.class, "index");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistance;

        // 2. Configuração da Limelight
        // Se necessário, use limelight.setPipeline(pipelineIndex); se o método estiver disponível.

        // 3. Telemetria de espera
        telemetry.addData("Status", "Inicializado. Aguardando Start.");
        telemetry.addData("Area Alvo (ta)", TARGET_AREA_PERCENT + "%");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // 4. Loop Principal
        while (opModeIsActive()) {
            boolean isDetected = false;

            if(sensorDistance.getDistance(DistanceUnit.MM) < distancia){
                isDetected = true;
                Motor.setPower(0);
            }else{
                Motor.setPower(0.5);
            }
            if (gamepad1.a){
                Motor.setPower(1);
            }

            telemetry.addData("Bola identificada: ", isDetected);

            // Obter os resultados mais recentes da Limelight
            LLResult results = limelight.getLatestResult();

            // CORREÇÃO: O método correto para verificar se um alvo foi encontrado
            // na classe LLResult do FTC SDK é getTargetFound().
            // Se o erro persistir, o método pode ser results.getTa() > 0.0,
            // que é o que o LLResult.getTargetFound() faz internamente.
            // Vamos tentar o método mais provável primeiro.

            // O método getTargetFound() está correto para a classe LLResult do FTC SDK.
            // O erro na imagem sugere que a versão do SDK do usuário pode ser diferente,
            // ou o método não está disponível na versão que ele está usando.
            // Como alternativa mais robusta, usaremos a verificação da área do alvo (ta).

            // Verificação alternativa: se a área do alvo (ta) for maior que zero, um alvo foi encontrado.
            if (results.getTa() > 0.0) {
                // Obter os valores tx, ty e ta do alvo principal
                double tx = results.getTx(); // Offset horizontal em graus
                double ty = results.getTy(); // Offset vertical em graus
                double ta = results.getTa(); // Área do alvo em porcentagem (0-100)

                // Usar 'ta' (área do alvo) como proxy para a distância.

                // Calcular o erro: (Área Alvo - Área Atual)
                double error = TARGET_AREA_PERCENT - ta;

                // Aplicar controle Proporcional (P)
                double motorPower = error * KP;

                // Limitar a potência do motor entre -1.0 e 1.0
                motorPower = Math.max(-1.0, Math.min(1.0, motorPower));

                // Aplicar a potência ao motor
                motorDistancia.setPower(motorPower);

                // Telemetria
                telemetry.addData("Status", "AprilTag Encontrada");
                telemetry.addData("tx (graus)", "%.2f", tx);
                telemetry.addData("ty (graus)", "%.2f", ty);
                telemetry.addData("ta (area %)", "%.2f", ta);
                telemetry.addData("Erro (ta)", "%.2f", error);
                telemetry.addData("Potência do Motor", "%.2f", motorPower);
            } else {
                // Nenhum alvo encontrado
                motorDistancia.setPower(0.0);
                telemetry.addData("Status", "Nenhum alvo AprilTag encontrado.");
                telemetry.addData("Potência do Motor", "0.00");
            }

            telemetry.update();
        }

        // Parar o motor ao sair do OpMode
        motorDistancia.setPower(0.0);
    }
}
