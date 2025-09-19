package org.firstinspires.ftc.teamcode.drive.camera;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class AprilTagLock extends LinearOpMode {
    PIDX pidx;
    PIDY pidy;
    public void runOpMode(){
        pidx = new PIDX();
        pidy = new PIDY();
    }
}
