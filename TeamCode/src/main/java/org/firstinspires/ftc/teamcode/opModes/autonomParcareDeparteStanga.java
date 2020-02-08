package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class autonomParcareDeparteStanga extends LinearOpMode {
    HardwareSkybot_V2 r = new HardwareSkybot_V2( true);
    @Override
    public void runOpMode() {
        r.Init(hardwareMap);
        while(!isStarted()){
            telemetry.addData("encDr", r.encDr);
            telemetry.addData("encSt", r.encSt);
            telemetry.addData("encSp", r.encSp);
            telemetry.update();
        }
        waitForStart();
        //r.stanga(700,0.3);
        r.startTime = System.currentTimeMillis();
        r.dreapta(700,0.5);
        r.fata(500,0.3);
        telemetry.addData("encDr", r.encDr);
        telemetry.addData("encSt", r.encSt);
        telemetry.addData("encSp", r.encSp);
        telemetry.addData("calculatedTicks", r.calculatedTicks);
        telemetry.update();
        sleep(3000);
        r.startTh = false;
    }
}
