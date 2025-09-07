package org.firstinspires.ftc.teamcode.drive.actuators;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Shooter{

    DcMotor motorbaixo;
    DcMotor motoralto;

    public Shooter(HardwareMap hardwareMap) {
        motorbaixo = hardwareMap.get(DcMotor.class, "motorb");
        motoralto = hardwareMap.get(DcMotor.class, "motora");
    }
    public void Shoot(double power){
        motorbaixo.setPower(power);
        motoralto.setPower(power);
    }
}
