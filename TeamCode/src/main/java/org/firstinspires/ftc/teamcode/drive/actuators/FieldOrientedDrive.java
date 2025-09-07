package org.firstinspires.ftc.teamcode.drive.actuators;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

public class FieldOrientedDrive {
    DcMotor leftFront, leftBack, rightFront, rightBack;
    IMU imu;

    public FieldOrientedDrive(HardwareMap hardwareMap) {
       leftFront = hardwareMap.get(DcMotor.class, "FL");
       leftBack = hardwareMap.get(DcMotor.class, "BL");
       rightFront = hardwareMap.get(DcMotor.class, "FR");
       rightBack = hardwareMap.get(DcMotor.class, "BR");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        Deadline gamepadRateLimit = new Deadline(500, TimeUnit.MILLISECONDS);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        ));
        imu.initialize(parameters);
    }


    public void movement(double lx, double ly, double rx, double drivepower, boolean resetIMU) {

        double max = Math.max(Math.abs(lx) + Math.abs(ly) + Math.abs(rx), 1);

        double drivePower = 1 - (0.5 * drivepower);

        if (resetIMU) {
            imu.resetYaw();
        }

        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double adjustedLx = ly * Math.sin(heading) + lx * Math.cos(heading);
        double adjustedLy = ly * Math.cos(heading) - lx * Math.sin(heading);

        leftFront.setPower(((adjustedLy + adjustedLx + rx) / max) * drivePower);
        leftBack.setPower(((adjustedLy - adjustedLx + rx) / max) * drivePower);
        rightFront.setPower(((adjustedLy - adjustedLx - rx) / max) * drivePower);
        rightBack.setPower(((adjustedLy + adjustedLx - rx) / max) * drivePower);
    }
}