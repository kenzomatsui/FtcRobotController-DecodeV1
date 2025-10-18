package org.firstinspires.ftc.teamcode.drive.autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.objects.FieldOrientedDrive;
import org.firstinspires.ftc.teamcode.drive.objects.Shooter;

// Anotação que registra esta classe como um OpMode Autônomo no sistema FTC.
// O nome "AutoRI30" será exibido no Driver Station e o grupo é "Autonomous".
@Autonomous(name = "AutoRI30", group = "Autonomous")
public class RI30HAuto extends LinearOpMode {

    // Declaração de um objeto Shooter, que provavelmente controla o mecanismo de disparo do robô.
    Shooter shooter;
    // Declaração de um objeto FieldOrientedDrive, que gerencia o sistema de direção do robô.
    FieldOrientedDrive fod;

    // O método runOpMode() é o ponto de entrada para o OpMode Linear Autônomo.
    // Todo o código autônomo é executado sequencialmente dentro deste método.
    @Override
    public void runOpMode(){
        // Inicializa o sistema de direção, passando o hardwareMap para acessar os dispositivos de hardware.
        fod = new FieldOrientedDrive(hardwareMap);
        // Inicializa o mecanismo de disparo, também passando o hardwareMap.
        shooter = new Shooter(hardwareMap);

        // Espera o botão 'Start' ser pressionado no Driver Station.
        waitForStart();

        // Define a potência do atirador para -0.5 (provavelmente para iniciar o atirador).
        shooter.Shoot(-0.5);
        // Pausa a execução por 2000 milissegundos (2 segundos).
        sleep(2000);
        // Ativa o mecanismo de intake (coleta).
        shooter.SetIntake();

        // Loop que continua enquanto o OpMode estiver ativo (após o 'Start' e antes do 'Stop').
        while(opModeIsActive()){
            // Define a potência do atirador para -0.5 novamente (mantém o atirador ligado).
            shooter.Shoot(-0.5);
            // Ativa o mecanismo de intake (coleta).
            shooter.SetIntake();
            // Pausa a execução por 8000 milissegundos (8 segundos).
            sleep(8000);
            // Move o robô lateralmente (strafe) com uma potência de 0.5.
            fod.Strafe(0.5);
        }
    }
}

