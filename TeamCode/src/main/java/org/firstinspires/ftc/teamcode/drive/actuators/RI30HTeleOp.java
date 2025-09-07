package org.firstinspires.ftc.teamcode.drive.actuators;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Teleop RI30H")
public class RI30HTeleOp extends OpMode {
    Shooter shooter;
    FieldOrientedDrive fod;

    public void init(){
        fod = new FieldOrientedDrive(hardwareMap);
        shooter = new Shooter(hardwareMap);
    }
    public void loop(){
        fod.movement(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_trigger, gamepad1.left_bumper);
        shooter.Shoot(-gamepad1.left_trigger);
        telemetry.addData("Speed shooter: ", -gamepad1.left_trigger);

        if (gamepad1.a){
            shooter.SetIntake();
        }
        if (gamepad1.y){
            shooter.SetStop();
        }
        if (gamepad1.b){
            shooter.Shoot(-0.74);
        }
        if (gamepad1.x){
            shooter.Shoot(0);
        }
    }
}
