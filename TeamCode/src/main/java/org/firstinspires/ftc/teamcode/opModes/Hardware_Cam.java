package org.firstinspires.ftc.teamcode.opModes;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.filters.DogeCVColorFilter;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;


public class Hardware_Cam extends LinearOpMode {

    public OpenCvCamera webcam;
    private int resWidth = 640, resHeight = 480;
    private Point p1 = new Point(0,0);//stanga sus
    private Point p2 = new Point(resHeight,resWidth);//dreapta jos
    public SkystoneDetectorModified skystoneDetectorModified = new SkystoneDetectorModified(p1, p2);
    public boolean isInitFinished = false;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();


    public Hardware_Cam(){
        skystoneDetectorModified.filter = new StoneDetector(p1, p2);
    }
    public Hardware_Cam(DogeCVColorFilter filter){
        skystoneDetectorModified.filter = filter;
    }
    public void Init(HardwareMap hard) {
        skystoneDetectorModified.stonesToFind = 1; 
        skystoneDetectorModified.useDefaults();
        skystoneDetectorModified.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA;
        int cameraMonitorViewId = hard.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hard.appContext.getPackageName());
        webcam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        webcam.openCameraDevice();
        webcam.setPipeline(skystoneDetectorModified);
        sleep(2000);
        webcam.startStreaming(resWidth, resHeight, OpenCvCameraRotation.UPRIGHT);;
        isInitFinished = true;
    }

    public void Init(HardwareMap hard, DogeCVColorFilter filter){
        skystoneDetectorModified.stonesToFind = 1;
        skystoneDetectorModified.useDefaults();
        skystoneDetectorModified.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA;
        skystoneDetectorModified.filter = filter;
        int cameraMonitorViewId = hard.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hard.appContext.getPackageName());
        webcam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        webcam.openCameraDevice();
        webcam.setPipeline(skystoneDetectorModified);
        sleep(2000);
        webcam.startStreaming(resWidth, resHeight, OpenCvCameraRotation.UPRIGHT);;
        isInitFinished = true;
    }
    public void startDetection(){
        webcam.startStreaming(resWidth, resHeight);
    }
    public void startDetection(DogeCVColorFilter filter){
        webcam.stopStreaming();
        sleep(2000);
        skystoneDetectorModified.filter = filter;
        sleep(2000);
        webcam.startStreaming(resWidth,resHeight,OpenCvCameraRotation.SIDEWAYS_LEFT);
        sleep(2000);
    }
    public void stopDetection(){
        webcam.stopStreaming();
    }

    @Override
    public void runOpMode(){}

}
