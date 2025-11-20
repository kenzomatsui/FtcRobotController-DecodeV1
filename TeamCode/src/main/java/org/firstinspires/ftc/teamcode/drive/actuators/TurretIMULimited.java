package org.firstinspires.ftc.teamcode.drive.actuators;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

@TeleOp(name = "Turret_IMU_Limited", group = "Testing")
public class TurretIMULimited extends LinearOpMode {

    private DcMotor turretMotor;
    private IMU imu;

    // Limites do movimento
    private static final double MIN_ANGLE = 0;      // grau mínimo
    private static final double MAX_ANGLE = 270;    // grau máximo

    @Override
    public void runOpMode() {

        turretMotor = hardwareMap.get(DcMotor.class, "turret");
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");

        // Configuração da IMU do Control Hub
        IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        imu.initialize(params);

        telemetry.addLine("IMU pronta. Aperte PLAY");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Obtém o heading atual da plataforma
            double angle = getHeading();

            // Controle manual (stick)
            double power = -gamepad1.right_stick_x;  // positivo = horário, negativo = anti-horário

            // Impedir rotação para fora dos limites
            if (angle <= MIN_ANGLE && power < 0) {
                // Está no limite mínimo e tentando passar → bloqueia
                power = 0;
            }

            if (angle >= MAX_ANGLE && power > 0) {
                // Está no limite máximo e tentando passar → bloqueia
                power = 0;
            }

            turretMotor.setPower(power);

            // Telemetria
            telemetry.addData("Heading (°)", angle);
            telemetry.addData("Power", power);
            telemetry.update();
        }
    }

    // Função que retorna o heading sempre entre 0° e 360°
    private double getHeading() {
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        // Converte para 0–360
        if (heading < 0) heading += 360;

        return heading;
    }
}
