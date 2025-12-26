package org.firstinspires.ftc.teamcode.drive.actuators;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Disabled
public class ServoTest extends OpMode {
    Servo servo;
    public void init(){
        servo = hardwareMap.get(Servo.class, "servoLeftName");
        telemetry.addData("Status: ", "Ready");
    }
    public void loop(){
        if (gamepad1.a){
            servo.setPosition(1);
        }
        if (gamepad1.b){
            servo.setPosition(0);
        }
    }
}
