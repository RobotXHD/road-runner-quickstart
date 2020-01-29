package org.firstinspires.ftc.teamcode.opModes;

import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.disnodeteam.dogecv.detectors.skystone.StoneDetector;
import com.disnodeteam.dogecv.filters.GrayscaleFilter;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
@Disabled
public class WebCam extends LinearOpMode {
    private OpenCvCamera webcam;
    private StoneDetector stoneDetector = new StoneDetector();
    private SkystoneDetector skystoneDetector = new SkystoneDetector();

    @Override
    public void runOpMode() throws InterruptedException {
        stoneDetector.useDefaults();
        stoneDetector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA;
        stoneDetector.speed = DogeCV.DetectionSpeed.VERY_FAST;
        stoneDetector.filter = new LeviColorFilter(LeviColorFilter.ColorPreset.RED, 110);
        stoneDetector.stonesToFind = 1;

        skystoneDetector.useDefaults();
        skystoneDetector.yellowFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.YELLOW, 110);
        skystoneDetector.blackFilter = new GrayscaleFilter(0, 50);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        webcam.setPipeline(stoneDetector);
        webcam.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT);
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.update();
        }
    }
}
