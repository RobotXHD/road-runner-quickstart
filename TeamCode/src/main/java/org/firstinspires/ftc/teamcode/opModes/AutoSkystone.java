package org.firstinspires.ftc.teamcode.opModes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;


@Autonomous
public class AutoSkystone extends LinearOpMode {
    Hardware_Cam cam = new Hardware_Cam();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    boolean isFront = false;
    long sysTime;
    int caz;
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        /** Daca esti in afara runOpMode hardwarewMap e gol; de aici vine nullpointer-ul */
        SampleMecanumDriveREVOptimized drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        drive.Init(hardwareMap);
        drive.pidPod.setPID(Automatizari_config.kpPod, Automatizari_config.kiPod, Automatizari_config.kdPod);
        drive.servoClamp.setPosition(configs.pozitie_servoClamp_prindere);
        waitForStart();
        drive.NOTDUCK = true;
        sysTime = System.currentTimeMillis();
        drive.extensieScissor(750);
        drive.homeScissor();
        /*
        while (opModeIsActive()) {
            drive.pidPod.setPID(Automatizari_config.kpPod, Automatizari_config.kiPod, Automatizari_config.kdPod);
            telemetry.addData("Pot", drive.potentiometruValue);
            telemetry.addData("Ceva",cam.skystoneDetectorModified.foundScreenPositions().get(0).x);

            if (cam.skystoneDetectorModified.foundScreenPositions().get(0).x >= 115) {
                telemetry.addData("Position", "Right");
                caz = 1;
            } else if (cam.skystoneDetectorModified.foundScreenPositions().get(0).x > 30) {
                telemetry.addData("Position", "Center");
                caz = 0;
            } else {
                telemetry.addData("Position", "Left");
                caz = -1;

            }
            telemetry.update();
            if(isFront && sysTime + 5000 < System.currentTimeMillis()){
                sysTime = System.currentTimeMillis();
                drive.goPodRulant(0);
                isFront = false;
            }
            else if(sysTime + 5000 < System.currentTimeMillis()){
                sysTime = System.currentTimeMillis();
                drive.goPodRulant(2500);
                isFront = true;
            }
        }
         */
        drive.stop = true;
    }
}