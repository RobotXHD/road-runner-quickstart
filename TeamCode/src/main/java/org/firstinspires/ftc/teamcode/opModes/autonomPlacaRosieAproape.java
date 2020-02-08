package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class autonomPlacaRosieAproape extends LinearOpMode {
    HardwareSkybot_V2 r = new HardwareSkybot_V2( true);
    @Override
    public void runOpMode() {
        r.Init(hardwareMap);
        while(!isStarted()){
            telemetry.addData("Status", "ready");
            telemetry.update();
        }
        waitForStart();
        r.startTime = System.currentTimeMillis();
        r.stanga(300, 0.5);
        r.spate(1000,0.3);
        r.servoPlatformaDr.setPosition(0);
        r.servoPlatformaSt.setPosition(1);
        sleep(2000);
        r.fata(1000 , 0.6);
        r.motorsf.setPower(0.6);
        r.motorss.setPower(0.6);
        sleep(1000);
        r.servoPlatformaDr.setPosition(1);
        r.servoPlatformaSt.setPosition(0.5);
        sleep(2000);
        r.dreapta(950, 0.5);
        sleep(3000);
        r.startTh = false;
    }
}
