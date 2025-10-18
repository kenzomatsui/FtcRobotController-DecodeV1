package org.firstinspires.ftc.teamcode.drive.objects;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Shooter{

    DcMotor motorbaixo;
    DcMotor motoralto;
    Servo servod;// 0
    Servo servoe;// 1

    public Shooter(HardwareMap hardwareMap) {
        motorbaixo = hardwareMap.get(DcMotor.class, "motorb");
        motoralto = hardwareMap.get(DcMotor.class, "motora");
        servod = hardwareMap.get(Servo.class, "servod");
        servoe = hardwareMap.get(Servo.class, "servoe");
    }
    public void Shoot(double power){
        motorbaixo.setPower(power);
        motoralto.setPower(power);
    }
    public void SetIntake(){
        servoe.setPosition(1);
        servod.setPosition(-1);
    }
    public void SetStop(){
        servoe.setPosition(0);
        servod.setPosition(0);
    }
}
