package org.firstinspires.ftc.teamcode.drive.actuators;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.objects.Intake;
import org.firstinspires.ftc.teamcode.drive.objects.ShooterObj;
@TeleOp
public class IntakeIndex extends OpMode {
    Intake intake;
    ShooterObj shooter;

    public void init(){
        intake = new Intake(hardwareMap);
        shooter = new ShooterObj(hardwareMap);
    }
    public void loop(){
        intake.Coleta(-gamepad1.left_trigger);
        shooter.detectBall(gamepad1.right_trigger);
    }
}
