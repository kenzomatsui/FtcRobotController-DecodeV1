package org.firstinspires.ftc.teamcode.drive.actuators;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Shooter extends OpMode {

    DcMotor motorbaixo;
    DcMotor motoralto;

    @Override
    public void init() {
        motorbaixo = hardwareMap.get(DcMotor.class, "motorb");
        motoralto = hardwareMap.get(DcMotor.class, "motora");
    }
    public void loop(){
        double power = gamepad1.left_stick_y;
        motorbaixo.setPower(power);
        motoralto.setPower(power);
    }
}
