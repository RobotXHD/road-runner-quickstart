package org.firstinspires.ftc.teamcode.opModes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.disnodeteam.dogecv.detectors.skystone.StoneDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

@Config
@Autonomous
public class AutoSkystone extends LinearOpMode {
    Hardware_Cam cam = new Hardware_Cam();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();
    PIDControllerAdevarat pidCam = new PIDControllerAdevarat(0,0,0);
    int caz;
    static int divLeft = 156, divRight = 38;
    double camPower = 0, power;
    boolean isCollected = false;
    @Override
    public void runOpMode() {
        cam.Init(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        while (!cam.isInitFinished){}
        packet.put("I'm already ", "here");
        dashboard.sendTelemetryPacket(packet);
        SampleMecanumDriveREVOptimized drive = new SampleMecanumDriveREVOptimized(hardwareMap); /** Daca esti in afara runOpMode hardwarewMap e gol; de aici vine nullpointer-ul */
        drive.Init(hardwareMap);
        drive.servoClamp.setPosition(configs.pozitie_servoClamp_prindere);
        while(!isStarted()){
            telemetry.addData("X", cam.skystoneDetectorModified.foundRectangles().get(0).x + (cam.skystoneDetectorModified.foundRectangles().get(0).width) / 2);
            telemetry.addData("Width", cam.skystoneDetectorModified.foundRectangles().get(0).width);
            telemetry.addData("height", cam.skystoneDetectorModified.foundRectangles().get(0).height);
            telemetry.update();
        }
        drive.extensieScissor();
        drive.servoClamp.setPosition(configs.pozitie_servoClamp_desprindere);
        sleep(1000);
        drive.homeScissor();

        sleep(5000);
        drive.servoClamp.setPosition(configs.pozitie_servoClamp_prindere);
        sleep(1000);

        drive.extensieScissor();
        drive.servoClamp.setPosition(configs.pozitie_servoClamp_desprindere);
        sleep(1000);
        drive.homeScissor();
        waitForStart();

    }
}