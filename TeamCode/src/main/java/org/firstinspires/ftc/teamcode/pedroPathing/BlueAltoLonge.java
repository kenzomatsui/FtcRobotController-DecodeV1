package org.firstinspires.ftc.teamcode.pedroPathing;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.objects.PedroPathingShooterController;
import org.firstinspires.ftc.teamcode.drive.objects.ShooterObjBlue;

@Autonomous
public class BlueAltoLonge extends OpMode {
    ShooterObjBlue shooter;
    DcMotor bl, fl, br, fr;
    DcMotor shoter;
    public void init(){
        shooter = new ShooterObjBlue(hardwareMap);
        shoter = hardwareMap.get(DcMotor.class,"RMTa");

        bl = hardwareMap.get(DcMotor.class,"BL");
        fl = hardwareMap.get(DcMotor.class,"FL");
        br = hardwareMap.get(DcMotor.class,"BR");
        fr = hardwareMap.get(DcMotor.class,"FR");
    }
    public void loop(){
        shoter.setPower(-0.95);
        sleep(5000);
        shooter.intake.setPower(-1);
        shoot3long();
        sleep(3000);
        bl.setPower(-1);
        fl.setPower(-1);
        br.setPower(1);
        fr.setPower(1);
        sleep(1000);
        bl.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        fr.setPower(0);
    }
    public void shoot3long(){
        shooter.indexer.setPower(1);
        sleep(600);
        shooter.indexer.setPower(0);
        sleep(600);
        shooter.indexer.setPower(1);
        sleep(600);
        shooter.indexer.setPower(0);
        sleep(600);
        shooter.indexer.setPower(1);
    }
}
