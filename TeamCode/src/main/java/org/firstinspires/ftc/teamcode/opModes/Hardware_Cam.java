package org.firstinspires.ftc.teamcode.opModes;

import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.filters.GrayscaleFilter;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import java.util.List;


public class Hardware_Cam<ArrayList> extends LinearOpMode {

    public OpenCvCamera webcam;
    private int resWidth = 640, resHeight = 480;
    private Point p1 = new Point(150,0);
    private Point p2 = new Point(resHeight,resWidth-150);
    public StoneDetectorModified stoneDetectorModified = new StoneDetectorModified(p1, p2);

    public Hardware_Cam(){}

    public void Init(HardwareMap hard) {
        stoneDetectorModified.stonesToFind = 1; 
        stoneDetectorModified.useDefaults();
        stoneDetectorModified.filter = new SkystoneDetector(p1, p2);
        stoneDetectorModified.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA;
        int cameraMonitorViewId = hard.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hard.appContext.getPackageName());
        webcam = new OpenCvWebcam(hard.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        webcam.setPipeline(stoneDetectorModified);
    }
    public void startDetection(){
        webcam.startStreaming(resWidth, resHeight);
    }
    public void stopDetection(){
        webcam.stopStreaming();
    }
    @Override
    public void runOpMode() {

    }
}
