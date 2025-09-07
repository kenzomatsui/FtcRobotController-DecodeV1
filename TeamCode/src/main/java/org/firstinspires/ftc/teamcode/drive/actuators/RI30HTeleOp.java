package org.firstinspires.ftc.teamcode.drive.actuators;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Teleop RI30H")
public class RI30HTeleOp extends OpMode {
    // Declaração de um objeto Shooter, que controla o mecanismo de disparo do robô.
    Shooter shooter;
    // Declaração do FieldOrientedDrive, que gerencia o sistema de direção orientado ao campo do robô.
    FieldOrientedDrive fod;

    @Override
    public void init(){
        // Inicializa o sistema de direção orientado ao campo, passando o hardwareMap para acessar os dispositivos de hardware.
        fod = new FieldOrientedDrive(hardwareMap);
        // Inicializa o mecanismo de disparo, também passando o hardwareMap.
        shooter = new Shooter(hardwareMap);
    }

    @Override
    public void loop(){
        // Controla o movimento do robô usando as entradas do gamepad1.
        // gamepad1.left_stick_x: movimento lateral (strafe).
        // -gamepad1.left_stick_y: movimento para frente/trás (negativo para frente, pois o y do joystick é invertido).
        // gamepad1.right_stick_x: rotação do robô.
        // gamepad1.right_trigger: controla a potência de condução (freio ou aceleração).
        // gamepad1.left_bumper: reseta a IMU (orientação do robô).
        fod.movement(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_trigger, gamepad1.left_bumper);

        // Controla a velocidade do atirador com base na entrada do gatilho esquerdo do gamepad1.
        shooter.Shoot(-gamepad1.left_trigger);
        // Envia a velocidade atual do atirador para a telemetria do Driver Station para depuração.
        telemetry.addData("Speed shooter: ", -gamepad1.left_trigger);

        // Verifica se o botão 'A' do gamepad1 foi pressionado.
        if (gamepad1.a){
            // Se 'A' for pressionado, ativa o mecanismo de intake (servo que passa a bola para os motores).
            shooter.SetIntake();
        }
        // Verifica se o botão 'Y' do gamepad1 foi pressionado.
        if (gamepad1.y){
            // Se 'Y' for pressionado, para o mecanismo de atirador/intake.
            shooter.SetStop();
        }
        // Verifica se o botão 'B' do gamepad1 foi pressionado.
        if (gamepad1.b){
            // Se 'B' for pressionado, define uma velocidade específica para o atirador (0.74 negativo).
            shooter.Shoot(-0.74);
        }
        // Verifica se o botão 'X' do gamepad1 foi pressionado.
        if (gamepad1.x){
            // Se 'X' for pressionado, para o atirador (velocidade 0).
            shooter.Shoot(0);
        }
    }
}

