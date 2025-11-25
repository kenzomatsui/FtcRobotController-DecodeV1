package org.firstinspires.ftc.teamcode.drive.actuators;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.actuators.MotorRPMWatcher;

@TeleOp(name = "VibrarControleRPM", group = "Testing")
public class TESTE extends LinearOpMode {

    private DcMotorEx motor;

    // Encoder do REV UltraPlanetary sem redução = 28 ticks * 4 = 112
    private static final double TICKS_PER_REV = 28 * 4;

    // RPM alvo para vibrar
    private static final double TARGET_RPM = 6000;

    @Override
    public void runOpMode() {

        motor = hardwareMap.get(DcMotorEx.class, "RMTa");

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Criar objeto que monitora RPM
        MotorRPMWatcher rpmWatcher = new MotorRPMWatcher(motor, TICKS_PER_REV, TARGET_RPM);

        telemetry.addLine("Pronto. Play para iniciar");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Controle do motor
            double power = gamepad1.right_trigger;
            motor.setPower(power);

            double rpm = rpmWatcher.getRPM();

            telemetry.addData("RPM", rpm);
            telemetry.addData("Alvo", rpmWatcher.getTargetRPM());
            telemetry.update();

            // Vibrar quando passar RPM
            if (rpmWatcher.isAboveTarget()) {
                gamepad1.rumble(500);  // vibra 0.5 segundos
            }
        }
    }
}
