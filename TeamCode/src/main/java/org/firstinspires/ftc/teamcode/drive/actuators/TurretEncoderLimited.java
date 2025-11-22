package org.firstinspires.ftc.teamcode.drive.actuators;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Turret_Encoder_Limited", group = "Testing")
public class TurretEncoderLimited extends LinearOpMode {

    private DcMotor turretMotor;

    // Configurações do Encoder do UltraPlanetary
    private static final double TICKS_PER_REV = 84.0;  // 28 * 3:1

    // Limites em graus
    private static final double MIN_ANGLE = 0;
    private static final double MAX_ANGLE = 270;

    @Override
    public void runOpMode() {

        turretMotor = hardwareMap.get(DcMotor.class, "turret");

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Sistema pronto! Aperte PLAY");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Lê ângulo atual baseado no encoder
            double angle = getTurretAngle();

            // Controle manual no stick direito
            double power = -gamepad1.right_stick_x;

            // Bloqueia ultrapassar os limites
            if (angle <= MIN_ANGLE && power < 0) {
                power = 0;
            }
            if (angle >= MAX_ANGLE && power > 0) {
                power = 0;
            }

            turretMotor.setPower(power);

            // Telemetria
            telemetry.addData("Encoder", turretMotor.getCurrentPosition());
            telemetry.addData("Ângulo (°)", angle);
            telemetry.addData("Power", power);
            telemetry.update();
        }
    }

    // Converte ticks de encoder para graus da plataforma
    private double getTurretAngle() {
        double ticks = turretMotor.getCurrentPosition();
        return (ticks / TICKS_PER_REV) * 360.0;
    }
}
