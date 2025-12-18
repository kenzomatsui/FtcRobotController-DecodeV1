package org.firstinspires.ftc.teamcode.drive.actuators;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.objects.PinpointServoController;

/**
 * Exemplo de OpMode para demonstrar o uso da classe PinpointServoController.
 *
 * Este OpMode deve ser usado em um robô com:
 * 1. Um sensor GoBilda Pinpoint configurado como "pinpoint".
 * 2. Dois servos configurados como "servo_left" e "servo_right".
 *
 * O robô tentará manter o ângulo de guinada (heading) em 0 graus,
 * movendo os servos em contraesterço para compensar a rotação.
 */
@TeleOp(name = "Pinpoint Servo Example", group = "TeamCode")
public class PinpointServoOpModeExample extends LinearOpMode {

    private PinpointServoController servoController = new PinpointServoController();

    // Nomes de configuração no arquivo de hardware
    private static final String PINPOINT_NAME = "pinpoint";
    private static final String SERVO_LEFT_NAME = "servo_left";
    private static final String SERVO_RIGHT_NAME = "servo_right";

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Inicializando PinpointServoController...");
        telemetry.update();

        // Inicializa o controlador de servos
        servoController.init(hardwareMap, PINPOINT_NAME, SERVO_LEFT_NAME, SERVO_RIGHT_NAME);

        telemetry.addData("Status", "Inicialização completa. Pressione Play.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // A lógica de controle é encapsulada no método update()
            servoController.update();

            // Exibe dados de telemetria para monitoramento
            telemetry.addData("Status", "Executando");
            telemetry.addData("Heading (Graus)", "%.2f", servoController.getCurrentHeading());
            telemetry.addData("Servo Esquerdo", "%.2f", servoController.getLeftPosition());
            telemetry.addData("Servo Direito", "%.2f", servoController.getRightPosition());
            telemetry.update();
        }
    }
}
