package org.firstinspires.ftc.teamcode.drive.actuators;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.objects.ColorControlledServo;

/**
 * Exemplo de OpMode para demonstrar o uso da classe ColorControlledServo.
 *
 * Este OpMode deve ser usado em um robô com:
 * - Um sensor de cor (NormalizedColorSensor) configurado como "color_sensor".
 * - Um servo configurado como "servo_control".
 *
 * O servo irá se mover automaticamente para a esquerda, direita ou centro
 * dependendo da cor (Verde, Roxo ou Nenhuma) detectada pelo sensor.
 */
@TeleOp(name = "Color Controlled Servo Example", group = "TeamCode")
@Disabled
public class ColorControlledServoOpModeExample extends LinearOpMode {

    private ColorControlledServo controller = new ColorControlledServo();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Inicializando ColorControlledServo...");
        telemetry.update();

        // Inicializa o controlador
        controller.init(hardwareMap);

        telemetry.addData("Status", "Inicialização completa. Pressione Play.");
        telemetry.addData("Instrução", "Coloque uma bola Verde, Roxa ou nada na frente do sensor.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // A lógica de controle é encapsulada no método update()
            controller.update();

            // Exibe dados de telemetria para monitoramento
            telemetry.addData("Status", "Executando");
            telemetry.addData("Cor Detectada", controller.getLastDetectedColor().toString());
            telemetry.addData("Posição do Servo", "%.2f", controller.getCurrentServoPosition());
            telemetry.update();
        }
    }
}
