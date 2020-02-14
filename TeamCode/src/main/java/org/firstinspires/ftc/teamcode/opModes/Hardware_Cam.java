package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.disnodeteam.dogecv.DogeCV;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


public class Hardware_Cam extends LinearOpMode {

    public OpenCvCamera webcam;
    private int resWidth = 640, resHeight = 480;
    private Point p1 = new Point(0,0);//stanga sus
    private Point p2 = new Point(resHeight-100,resWidth-400);//dreapta jos
    public SkystoneDetectorModified skystoneDetectorModified = new SkystoneDetectorModified(p1, p2);
    public boolean isInitFinished = false;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();


    public Hardware_Cam(){}
    public void Init(HardwareMap hard) {

        skystoneDetectorModified.stonesToFind = 1; 
        skystoneDetectorModified.useDefaults();
        skystoneDetectorModified.filter = new SkystoneDetector(p1, p2);
        skystoneDetectorModified.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA;

        int cameraMonitorViewId = hard.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hard.appContext.getPackageName());
        webcam = new OpenCvWebcam(hard.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        packet.put("Even though I should be ", "here");
        dashboard.sendTelemetryPacket(packet);
        webcam.setPipeline(skystoneDetectorModified);
        sleep(2000);
        webcam.startStreaming(resWidth, resHeight, OpenCvCameraRotation.SIDEWAYS_LEFT);
        isInitFinished = true;
    }
    public void startDetection(){
        webcam.startStreaming(resWidth, resHeight);
    }
    public void stopDetection(){
        webcam.stopStreaming();
    }

    @Override
    public void runOpMode(){}

}
