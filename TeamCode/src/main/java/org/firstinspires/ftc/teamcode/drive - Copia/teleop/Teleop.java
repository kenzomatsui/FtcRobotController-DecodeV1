package org.firstinspires.ftc.teamcode.drive.teleop;

import static android.os.SystemClock.sleep;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.drive.actuators.Intake;
import org.firstinspires.ftc.teamcode.drive.actuators.Outtake;

import java.util.concurrent.TimeUnit;

@TeleOp
public class Teleop extends OpMode {

    //INTAKE
    Intake intake;

    Servo rotate;
    Servo garra;
    Servo pleft;
    Servo pright;
    Servo lright;
    Servo lleft;

    //OUTTAKE
    Outtake outtake;

    DcMotor poliaright;
    DcMotor polialeft;
    Servo Bright;
    Servo Bleft;
    Servo garrinha;
    double ticks = 2750;
    double ticks2 = 2200;
    double ticks3 = 338;
    double newTarget;

    //MOVIMENTACAO

    DcMotor frontLeft, frontRight, backLeft, backRight;
    IMU imu;


    @Override
    public void init() {

        //INTAKE
        lright = hardwareMap.get(Servo.class, "lright");
        lleft = hardwareMap.get(Servo.class, "lleft");
        rotate = hardwareMap.get(Servo.class, "rotate");
        garra = hardwareMap.get(Servo.class, "garra");
        pleft = hardwareMap.get(Servo.class, "pleft");
        pright = hardwareMap.get(Servo.class, "pright");

        intake = new Intake(hardwareMap);

        //OUTTAKE
        poliaright = hardwareMap.get(DcMotor.class, "poliaright");
        polialeft = hardwareMap.get(DcMotor.class, "polialeft");
        Bright = hardwareMap.get(Servo.class, "bright");
        Bleft = hardwareMap.get(Servo.class, "bleft");
        garrinha = hardwareMap.get(Servo.class, "garrinha");

        poliaright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        poliaright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        poliaright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        polialeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        outtake = new Outtake(hardwareMap);

        //MOVIMENTACAO
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");

        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Hardware: ", "Initialized");

        lright.setPosition(0.85);
        lleft.setPosition(0.35);
        rotate.setPosition(0.7);
        pleft.setPosition(0.8);
        pright.setPosition(0.2);
        garra.setPosition(0.3);
    }
        public void loop(){
        telemetry.addData("PoliaLeft: ", polialeft.getCurrentPosition());
        telemetry.addData("PoliaRight: ", poliaright.getCurrentPosition());
            //INTAKE
            if (gamepad2.dpad_down) {
                intake.retractsIntake();
            }
            if (gamepad2.a) {
                garra.setPosition(0.3);
            }
            if (gamepad2.b){
                garra.setPosition(0.5);
            }
            if (gamepad2.left_bumper){
                rotate.setPosition(1);
            }
            if (gamepad2.right_bumper){
                rotate.setPosition(0.5);
            }
            if (gamepad1.left_trigger > 0.1) {
                polialeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                poliaright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            if (gamepad2.dpad_up) {
                intake.extendsIntake();
            }

            //OUTTAKE

            if (gamepad1.dpad_up) {
                outtake.basketSet();
            }
            if (gamepad1.dpad_left) {
                outtake.wallClipSet();
            }
            if (gamepad1.dpad_right){
                outtake.wranglerClipSet();
            }
            if (gamepad1.dpad_down) {
                outtake.slidesDown();
            }
            if (gamepad1.y) {
                Bright.setPosition(1);
                Bleft.setPosition(0);
            }
            if (gamepad1.x) {
                Bright.setPosition(0.4);
                Bleft.setPosition(0.6);
            }
            if (gamepad1.b) {
                garrinha.setPosition(0.4);
            }
            if (gamepad1.a) {
                garrinha.setPosition(0);
            }

            //MOVIMENTACAO
            Deadline gamepadRateLimit = new Deadline(500, TimeUnit.MILLISECONDS);

            IMU imu = hardwareMap.get(IMU.class, "imu");
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
            ));
            imu.initialize(parameters);
            double lx = -gamepad1.left_stick_x;
            double ly = gamepad1.left_stick_y;
            double rx = -gamepad1.right_stick_x;

            double max = Math.max(Math.abs(lx) + Math.abs(ly) + Math.abs(rx), 1);

            double drivePower = 1 - (0.5 * gamepad1.right_trigger);

            if(gamepad1.left_bumper) imu.resetYaw();

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double adjustedLx = +ly * Math.sin(heading) + lx * Math.cos(heading);
            double adjustedLy = ly * Math.cos(heading) - lx * Math.sin(heading);

            frontLeft.setPower(((adjustedLy + adjustedLx + rx) / max) * drivePower);
            backLeft.setPower(((adjustedLy - adjustedLx + rx) / max) * drivePower);
            frontRight.setPower(((adjustedLy - adjustedLx - rx) / max) * drivePower);
            backRight.setPower(((adjustedLy + adjustedLx - rx) / max) * drivePower);
            telemetry.update();

        }
}

/*
CONFIGURAÇÃO DO DRIVER

CONTROL HUB:
Motores
0 - odor (backleft)
1 - odom (backright)
2 - odol (frontleft)
3 - FR (frontright)

Servos
0 - lright
1 - lleft
2 - garra
3 - rotate
4 - pleft
5 - pright

EXPANSION HUB:
Motores
0 - nada
1 - nada
2 - poliaright
3 - polialeft

Servos
0 -
1 -
2 -
3 - garrinha
4 - bleft
5 - bright
 */