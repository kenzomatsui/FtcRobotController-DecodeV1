package org.firstinspires.ftc.teamcode.drive.actuators;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.drive.objects.FieldOrientedDrive;

public class MovFOD extends OpMode {
    FieldOrientedDrive fod;

    public void init() {
        fod = new FieldOrientedDrive(hardwareMap);
    }

    public void loop() {
        fod.movement(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_trigger, gamepad1.right_bumper);
    }
}