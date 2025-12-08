package org.firstinspires.ftc.teamcode.drive.teleop;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.objects.FieldOrientedDrive;
import org.firstinspires.ftc.teamcode.drive.objects.Intake;
import org.firstinspires.ftc.teamcode.drive.objects.ShooterObj;

@TeleOp
public class ProtTeleOpBlue extends OpMode {
    FieldOrientedDrive fod;
    ShooterObj shooter;
    Intake intake;


    public void init() {
        fod = new FieldOrientedDrive(hardwareMap);
        shooter = new ShooterObj(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter.update();
    }

    public void loop() {
        fod.movement(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_bumper);
        shooter.aimAndShoot();
        shooter.voltaIndex(gamepad2.left_trigger);
        shooter.detectBall();
        shooter.Shoot(gamepad1.right_trigger);
        intake.Coleta(-gamepad1.left_trigger, -gamepad1.right_trigger);
        //telemetry.addData("Power: ", shooter.get_power());
        telemetry.update();
    }
}

/* CONFIGURAÇÃO DO DRIVER
 * Control hub:
 * 0 - RMX
 * 1 - index
 * 2 - RMTa
 * 3 - intake
 *
 * I2c:
 * 0 - imu
 * 1 - pinpoint
 * 2 - sensor_distance
 *
 * USB 2.0: limelight
 *
 * Expansion hub:
 * 0 - BL
 * 1 - BR
 * 2 - FL
 * 3 - FR
 */
