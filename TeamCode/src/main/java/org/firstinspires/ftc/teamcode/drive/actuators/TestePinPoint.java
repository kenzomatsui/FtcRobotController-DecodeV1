package org.firstinspires.ftc.teamcode.drive.actuators;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PinpointOdometry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name = "TestePinPoint (Java)")
public class TestePinPoint extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Inicializa o objeto Pinpoint a partir do hardwareMap
        PinpointOdometry pinpoint = hardwareMap.get(PinpointOdometry.class, "pinpoint");

        // Configura resolução do encoder (ticks por mm)
        // Para o GoBILDA 4-Bar Odometry Pod, a conversão já é conhecida:
        // Você pode ajustar conforme o datasheet do seu pod
        double ticksPerMM = 2000.0 / 35.0;  // exemplo -> revise para seu pod real
        pinpoint.setEncoderResolution(ticksPerMM, DistanceUnit.MM);


        // Espera o start
        waitForStart();

        while (opModeIsActive()) {
            // Atualiza os dados do odômetro
            pinpoint.update();

            // Pega posição X, Y e orientação
            double x = pinpoint.getPosition(DistanceUnit.CM).getX(DistanceUnit.CM);
            double y = pinpoint.getPosition(DistanceUnit.CM).getY(DistanceUnit.CM);
            double heading = pinpoint.getHeading(AngleUnit.DEGREES);

            // Mostra no telemetry
            telemetry.addData("Distância em X (cm)", x);
            telemetry.addData("Distância em Y (cm)", y);
            telemetry.addData("Orientação (graus)", heading);
            telemetry.update();
        }
    }
}
