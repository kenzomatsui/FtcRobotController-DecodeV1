package org.firstinspires.ftc.teamcode.drive.actuators;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "RPM_ThroughBore", group = "Testing")
public class REVTroughBore extends LinearOpMode {

    // Motor com encoder externo Through Bore
    private DcMotorEx motor;

    // Through Bore Encoder Specs
    // No modo quadrature → 8192 counts por rotação
    private static final double TICKS_PER_REV = 8192.0;

    @Override
    public void runOpMode() {

        motor = hardwareMap.get(DcMotorEx.class, "RMTa");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Pronto! Aperte Play");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Velocidade bruta em ticks/segundo
            double ticksPerSecond = motor.getVelocity();

            // Converter para RPM
            double rpm = (ticksPerSecond * 60.0) / TICKS_PER_REV;

            telemetry.addData("Ticks por segundo", ticksPerSecond);
            telemetry.addData("RPM", rpm);
            telemetry.update();
        }
    }
}