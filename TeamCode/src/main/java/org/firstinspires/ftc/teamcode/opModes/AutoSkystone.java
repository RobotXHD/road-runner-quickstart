package org.firstinspires.ftc.teamcode.opModes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.disnodeteam.dogecv.detectors.skystone.StoneDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

@Config
@Autonomous
public class AutoSkystone extends LinearOpMode {
    Hardware_Cam cam = new Hardware_Cam();
    Hardware_Scissor_V1 scissor = new Hardware_Scissor_V1();
    StoneDetector stoneDetector = new StoneDetector();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();
    PIDControllerAdevarat pidCam = new PIDControllerAdevarat(0,0,0);
    int caz;
    static int divLeft = 156, divRight = 38;
    double camPower = 0, power;
    boolean isCollected = false;
    SampleMecanumDriveREVOptimized drive;
    @Override
    public void runOpMode() {
        stoneDetector.useDefaults();
        cam.Init(hardwareMap);
        scissor.Init(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        while (!cam.isInitFinished){}
        packet.put("I'm already ", "here");
        dashboard.sendTelemetryPacket(packet);
        drive = new SampleMecanumDriveREVOptimized(hardwareMap); /** Daca esti in afara runOpMode hardwarewMap e gol; de aici vine nullpointer-ul */
        pidCam.setSetpoint(100);
        pidCam.setPID(camConfig.kp, camConfig.ki, camConfig.kd);
        pidCam.enable();
        drive.Init(hardwareMap);
        cam.startDetection(new org.firstinspires.ftc.teamcode.opModes.StoneDetector(480, 640));
        while(!isStarted()){
            telemetry.addData("pot", drive.potentiometruValue);
            telemetry.update();
        }
        waitForStart();
        drive.goPodRulant(1500);
        drive.goPodRulant(0);
    }
}