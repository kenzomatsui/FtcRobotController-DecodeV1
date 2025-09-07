package org.firstinspires.ftc.teamcode.drive.autonomus;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.actuators.FieldOrientedDrive;
import org.firstinspires.ftc.teamcode.drive.actuators.Shooter;

public class RI30HAuto extends LinearOpMode {
    Shooter shooter;
    FieldOrientedDrive fod;

    public void runOpMode(){
        fod = new FieldOrientedDrive(hardwareMap);
        shooter = new Shooter(hardwareMap);
        while(opModeIsActive()){
            shooter.Shoot(1);
            sleep(6000);
            fod.Strafe(0.5);
        }
    }
}
