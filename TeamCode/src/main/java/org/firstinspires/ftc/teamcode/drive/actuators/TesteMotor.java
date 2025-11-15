package org.firstinspires.ftc.teamcode.drive.actuators;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Teste Motor", group="Testes")
@Disabled
public class TesteMotor extends OpMode {

    DcMotor motor, motor1;

    @Override
    public void init() {
        // nome do motor deve ser igual ao configurado no Driver Hub
        motor = hardwareMap.get(DcMotor.class, "RMTa");
        motor1 = hardwareMap.get(DcMotor.class, "RMX");
        telemetry.addData("Status", "Inicializado");
    }

    @Override
    public void loop() {
        // controlado pelo gatilho direito e esquerdo do gamepad
        double power = gamepad1.right_trigger - gamepad1.left_trigger;
        motor.setPower(power);
        motor1.setPower(-power);

        telemetry.addData("Potência", power);
        telemetry.update();
    }
}