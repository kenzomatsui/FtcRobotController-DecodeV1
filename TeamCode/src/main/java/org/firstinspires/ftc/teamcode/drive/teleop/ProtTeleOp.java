package org.firstinspires.ftc.teamcode.drive.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.objects.FieldOrientedDrive;
import org.firstinspires.ftc.teamcode.drive.objects.ShooterObj;

@TeleOp
public class ProtTeleOp extends OpMode {
    FieldOrientedDrive fod; //Classe da Movimentação do Chassi
    ShooterObj shooter;


    public void init() {
        shooter = new ShooterObj(hardwareMap);
        fod = new FieldOrientedDrive(hardwareMap);
    }

    public void loop() {
        shooter.update();
        shooter.aimAndShoot();
        shooter.Shoot(gamepad1.right_trigger);
        fod.movement(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_trigger, gamepad1.right_bumper);

    }
}
