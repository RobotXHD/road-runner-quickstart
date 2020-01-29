package org.firstinspires.ftc.teamcode.opModes;

import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.filters.DogeCVColorFilter;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
@Disabled
public class AutoLevi extends LinearOpMode {
    OpenCvCamera webcam;
    StoneDetectorModified stoneDetectorModified = new StoneDetectorModified(new Point(), new Point());
    int aMini = 120, aMaxi = 157;
    int bMini = 162, bMaxi = 209;
    double systime;
    @Override
    public void runOpMode()
    {
        stoneDetectorModified.useDefaults();
        stoneDetectorModified.filter = new YellowStoneDetector();
        stoneDetectorModified.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        webcam.setPipeline(stoneDetectorModified);
        webcam.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT);
        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.a){
               aMaxi += 2;
               sleep(200);
               //webcam.stopStreaming();
               //stoneDetectorModified.filter = new Levii();
               //webcam.startStreaming(640, 480);
            }
            if(gamepad1.x){
                aMaxi -= 2;
                sleep(200);
                //webcam.stopStreaming();
                //stoneDetectorModified.filter = new Levii();
                //webcam.startStreaming(640, 480);
            }
            if(gamepad1.b){
                bMaxi += 2;
                sleep(200);
                //webcam.stopStreaming();
                //stoneDetectorModified.filter = new Levii();
                //webcam.startStreaming(640, 480);
            }
            if(gamepad1.y){
                bMaxi -= 2;
                sleep(200);
                //webcam.stopStreaming();
                //stoneDetectorModified.filter = new Levii();
                //webcam.startStreaming(640, 480);
            }
            if(gamepad1.dpad_up){
                aMini += 2;
                sleep(200);
                //webcam.stopStreaming();
                //stoneDetectorModified.filter = new Levii();
                //webcam.startStreaming(640, 480);
            }
            if(gamepad1.dpad_down){
                aMini -= 2;
                sleep(200);
                //webcam.stopStreaming();
                //stoneDetectorModified.filter = new Levii();
                //webcam.startStreaming(640, 480);
            }
            if(gamepad1.dpad_right){
                bMini += 2;
                sleep(200);
                //webcam.stopStreaming();
                //stoneDetectorModified.filter = new Levii();
                //webcam.startStreaming(640, 480);
            }
            if( gamepad1.dpad_left){
                bMini -= 2;
                sleep(200);
                //webcam.stopStreaming();
                //stoneDetectorModified.filter = new Levii();
                //webcam.startStreaming(640, 480);
            }
           /* telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());*/

            telemetry.addData("X sau Y: ", stoneDetectorModified.foundRectangles());
            telemetry.addData("Ceva: ", stoneDetectorModified.foundScreenPositions());
            telemetry.update();
        }
    }

    public class Levii extends DogeCVColorFilter {
        @Override
        public void process(Mat input, Mat mask) {
            Mat lab = new Mat(input.size(), 0);
            Imgproc.GaussianBlur(input,input,new Size(7,7),0);
            Imgproc.cvtColor(input, lab, Imgproc.COLOR_RGB2Lab);
            Core.inRange(lab, new Scalar(0, aMini, bMini), new Scalar(255, aMaxi, bMaxi), lab);
            Imgproc.GaussianBlur(lab, mask, new Size(3,3), 0);
            lab.release();
            input.release();
        }
    }
}
