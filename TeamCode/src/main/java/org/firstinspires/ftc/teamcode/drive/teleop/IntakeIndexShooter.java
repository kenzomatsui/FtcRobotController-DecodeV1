package org.firstinspires.ftc.teamcode.drive.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.objects.Intake;
import org.firstinspires.ftc.teamcode.drive.objects.ShooterObj;
@TeleOp
@Disabled
public class IntakeIndexShooter extends OpMode {
    Intake intake;
    ShooterObj shooter;

    public void init(){
        intake = new Intake(hardwareMap);
        shooter = new ShooterObj(hardwareMap);
    }
    public void loop(){
        intake.Coleta(-gamepad1.left_trigger, -gamepad1.right_trigger);
        shooter.aimAndShoot();
        telemetry.addData("Sensor: ", shooter.sensorDistance.getDistance(DistanceUnit.MM));
    }
}
