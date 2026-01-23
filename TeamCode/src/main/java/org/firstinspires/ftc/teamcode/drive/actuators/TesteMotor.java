package org.firstinspires.ftc.teamcode.drive.actuators;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Teste Motor", group="Testes")
public class TesteMotor extends OpMode {

    DcMotor motor, motor1;
    Servo servo, servo1;

    @Override
    public void init() {
        // nome do motor deve ser igual ao configurado no Driver Hub
        motor = hardwareMap.get(DcMotor.class, "r");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1 = hardwareMap.get(DcMotor.class, "l");
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        servo = hardwareMap.get(Servo.class,"s");
        servo1 = hardwareMap.get(Servo.class,"s1");
        telemetry.addData("Status", "Inicializado");
        gamepad1.rumble(500);
    }

    @Override
    public void loop() {
        // controlado pelo gatilho direito e esquerdo do gamepad
        double power = gamepad1.right_trigger - gamepad1.left_trigger;
        motor.setPower(power);
        motor1.setPower(power);

        if (gamepad1.a){
            servo.setPosition(0);
            servo1.setPosition(1);
        }
        if (gamepad1.b){
            servo.setPosition(0.58);
            servo1.setPosition(0.5);
        }
        if (gamepad1.x){
            servo.setPosition(1);
            servo1.setPosition(0);
        }

        telemetry.addData("Potência", power);
        telemetry.update();
    }
}