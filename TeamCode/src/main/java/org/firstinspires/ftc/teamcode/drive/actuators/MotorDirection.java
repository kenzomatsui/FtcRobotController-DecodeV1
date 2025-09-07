package org.firstinspires.ftc.teamcode.drive.actuators;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class MotorDirection extends OpMode {
    DcMotor backLeft;
    DcMotor backRight;
    DcMotor frontLeft;
    DcMotor frontRight;
    public void init() {
        backLeft = hardwareMap.dcMotor.get("BL");
        backRight = hardwareMap.dcMotor.get("BR");
        frontLeft = hardwareMap.dcMotor.get("FL");
        frontRight = hardwareMap.dcMotor.get("FR");

        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Reversão de valores
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Hardware: ", "Initialized");
        telemetry.addData("Status: ", "Initialized");
        telemetry.update();
    }
    public void loop(){
        if (gamepad1.a){
            backLeft.setPower(0.2);
        }
        telemetry.addLine("BackLeft: A");

        if (gamepad1.b){
            backRight.setPower(0.2);
        }
        telemetry.addLine("BackRight: B");

        if (gamepad1.x){
            frontLeft.setPower(0.2);
        }
        telemetry.addLine("FrontLeft: X");

        if (gamepad1.y){
            frontRight.setPower(0.2);
        }
        telemetry.addLine("FrontRight: Y");
    }
}
