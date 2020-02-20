package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static java.lang.Math.PI;


public class DubiousTest extends LinearOpMode {
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
        r.stanga(600, 0.5);
        r.spate(1000,0.3);
        r.servoPlatformaDr.setPosition(0);
        r.servoPlatformaSt.setPosition(1);
        sleep(2000);
        r.fata(900 , 0.6);
        r.servoPlatformaDr.setPosition(1);
        r.servoPlatformaSt.setPosition(0.5);
        sleep(2000);
        r.dreapta(800, 0.5);
        r.spate(550, 0.3);
        r.dreapta(550, 0.5);
        //r.stanga(700,0.3);
        //r.dreapta(700,0.5);
        //r.fata(500,0.3);
        telemetry.addData("encDr", r.encDr);
        telemetry.addData("encSt", r.encSt);
        telemetry.addData("encSp", r.encSp);
        telemetry.addData("calculatedTicks", r.calculatedTicks);
        telemetry.update();
        sleep(3000);
        r.startTh = false;
    }
}
