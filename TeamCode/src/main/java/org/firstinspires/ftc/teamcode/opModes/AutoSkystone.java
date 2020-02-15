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
    Hardware_Scissor_V1 scissor = new Hardware_Scissor_V1();
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
        scissor.Init(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        while (!cam.isInitFinished){}
        packet.put("I'm already ", "here");
        dashboard.sendTelemetryPacket(packet);
        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap); /** Daca esti in afara runOpMode hardwarewMap e gol; de aici vine nullpointer-ul */
        pidCam.setSetpoint(100);
        pidCam.setPID(camConfig.kp, camConfig.ki, camConfig.kd);
        pidCam.enable();
        while(!isStarted()){
            telemetry.addData("X", cam.skystoneDetectorModified.foundRectangles().get(0).x + (cam.skystoneDetectorModified.foundRectangles().get(0).width) / 2);
            telemetry.addData("Width", cam.skystoneDetectorModified.foundRectangles().get(0).width);
            telemetry.addData("height", cam.skystoneDetectorModified.foundRectangles().get(0).height);
            telemetry.update();
        }
        waitForStart();
        scissor.startColect();
        while(opModeIsActive()) {
            while (!isCollected) {
                if(cam.skystoneDetectorModified.foundRectangles().get(0).width < 50){
                    power = 0.2;
                }
                else{
                    power = 0.4;
                }
                isCollected = scissor.touchGheara.isPressed();
                pidCam.setPID(camConfig.kp, camConfig.ki, camConfig.kd);
                camPower = pidCam.performPID(cam.skystoneDetectorModified.foundRectangles().get(0).x + (cam.skystoneDetectorModified.foundRectangles().get(0).width) / 2);
                drive.setMotorPowers(power-camPower, power-camPower, power+camPower, power+camPower);
                telemetry.addData("X", cam.skystoneDetectorModified.foundRectangles().get(0).x + (cam.skystoneDetectorModified.foundRectangles().get(0).width) / 2);
                telemetry.update();
            }
            drive.setMotorPowers(0,0,0,0);
            telemetry.addData("found", cam.skystoneDetectorModified.isDetected());
            telemetry.addData("touchGheara ",scissor.touchGheara.isPressed());
            telemetry.update();
        }
    }
}