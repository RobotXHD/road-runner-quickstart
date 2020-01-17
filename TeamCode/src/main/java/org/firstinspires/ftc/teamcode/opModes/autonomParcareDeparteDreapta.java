package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class autonomParcareDeparteDreapta extends LinearOpMode {
    HardwareSkybot_V2 r = new HardwareSkybot_V2( true);
    @Override
    public void runOpMode() {
        r.Init(hardwareMap);
        while(!isStarted()){
            telemetry.addData("Status", "ready");
            telemetry.update();
        }
        waitForStart();
        //r.fata(500,0.3);
        r.startTime = System.currentTimeMillis();
        r.stanga(700,0.5);
        r.fata(500,0.3);
        sleep(3000);
        r.startTh = false;
    }
}
