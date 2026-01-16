package org.firstinspires.ftc.teamcode.drive.actuators;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Exemplo de OpMode para demonstrar o uso da classe PinpointMotorController.
 *
 * Este OpMode deve ser usado em um robô com:
 * 1. Um sensor GoBilda Pinpoint configurado como "pinpoint".
 * 2. Um motor com redução 15:1 configurado como "steering_motor".
 *
 * O motor atuará em contraesterço e alternará entre os limites da zona definida.
 */
@TeleOp(name = "Pinpoint Motor Example", group = "TeamCode")
public class PinpointMotorOpModeExample extends LinearOpMode {

    private PinpointMotorController motorController = new PinpointMotorController();

    // Nomes de configuração no arquivo de hardware
    private static final String PINPOINT_NAME = "pinpoint";
    private static final String MOTOR_NAME = "RMX";

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Inicializando PinpointMotorController...");
        telemetry.update();

        // Inicializa o controlador de motor
        motorController.init(hardwareMap, PINPOINT_NAME, MOTOR_NAME);

        // Configurações opcionais
        motorController.setLimits(-150.0, 150.0); // Define a zona de graus
        motorController.setAutoReverse(true);    // Ativa a inversão automática nos limites

        telemetry.addData("Status", "Inicialização completa. Pressione Play.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // A lógica de controle é encapsulada no método update()
            motorController.update();

            // Exibe dados de telemetria para monitoramento
            telemetry.addData("Status", "Executando");
            telemetry.addData("Heading (Graus)", "%.2f", motorController.getCurrentHeading());
            telemetry.addData("Ângulo do Motor", "%.2f", motorController.getMotorAngle());
            telemetry.addData("Poder do Motor", "%.2f", motorController.getMotorPower());
            telemetry.update();
        }
    }
}
