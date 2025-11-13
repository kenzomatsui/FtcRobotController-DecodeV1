package org.firstinspires.ftc.teamcode.drive.actuators;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class AllMotors extends OpMode {
    DcMotor a, b, c, d, e, f, g, h;
    public void init(){
        a = hardwareMap.get(DcMotor.class, "a");
        b = hardwareMap.get(DcMotor.class, "b");
        c = hardwareMap.get(DcMotor.class, "c");
        d = hardwareMap.get(DcMotor.class, "d");
        e = hardwareMap.get(DcMotor.class, "e");
        f = hardwareMap.get(DcMotor.class, "f");
        g = hardwareMap.get(DcMotor.class, "g");
        h = hardwareMap.get(DcMotor.class, "h");
    }
    public void loop(){
        double power = gamepad1.right_trigger;

        a.setPower(power);
        b.setPower(power);
        c.setPower(power);
        d.setPower(power);
        e.setPower(power);
        f.setPower(power);
        g.setPower(power);
        h.setPower(power);
    }
}
