package org.firstinspires.ftc.teamcode.drive.actuators;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Exemplo de OpMode TeleOp utilizando a classe CounterSteerControl.
 * Este código demonstra como configurar o motor, o Pinpoint e a lógica de controle.
 */
@TeleOp(name = "CounterSteer TeleOp Example", group = "Examples")
public class CounterSteerTeleOp extends OpMode {

    private GoBildaPinpointDriver pinpoint;
    private DcMotorEx steeringMotor;
    private CounterSteerControl counterSteerControl;

    @Override
    public void init() {
        // 1. Inicializar o hardware
        // Certifique-se de que os nomes coincidem com a configuração no Driver Station
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        steeringMotor = hardwareMap.get(DcMotorEx.class, "RMX");

        // 2. Configurar o motor de direção
        steeringMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // 3. Inicializar a classe de controle
        counterSteerControl = new CounterSteerControl(steeringMotor, pinpoint);

        // 4. Configurações customizadas
        // PID para motor 15:1 (ajuste conforme necessário para seu mecanismo)
        counterSteerControl.setPID(0.05, 0.0, 0.005);

        // Define a zona de atuação (ex: de -60 a 60 graus)
        counterSteerControl.setLimits(-60.0, 60.0);

        // Ativa a inversão automática quando chegar nos limites
        counterSteerControl.setAutoReverse(true);

        // Reseta o encoder do motor para garantir que 0 seja a posição central
        counterSteerControl.resetEncoder();

        telemetry.addData("Status", "Inicializado");
    }

    @Override
    public void loop() {
        // 1. Atualizar a lógica de controle
        // O método update() cuida da leitura do Pinpoint, cálculo PID e limites
        counterSteerControl.update();

        // 2. Controles manuais opcionais (via Gamepad)
        if (gamepad1.a) {
            counterSteerControl.setAutoReverse(true);
        } else if (gamepad1.b) {
            counterSteerControl.setAutoReverse(false);
        }

        // 3. Telemetria para monitoramento e debug
        telemetry.addData("--- Pinpoint ---", "");
        telemetry.addData("Heading", "%.2f deg", pinpoint.getHeading(AngleUnit.DEGREES));

        telemetry.addData("--- Steering Motor ---", "");
        telemetry.addData("Current Angle", "%.2f deg", counterSteerControl.getMotorAngle());
        telemetry.addData("Power", "%.2f", steeringMotor.getPower());

        telemetry.addData("--- Status ---", "");
        telemetry.addData("Auto Reverse", gamepad1.a ? "Ativado" : (gamepad1.b ? "Desativado" : "Mantido"));

        telemetry.update();
    }

    @Override
    public void stop() {
        // Para o motor ao encerrar o OpMode
        steeringMotor.setPower(0);
    }
}
