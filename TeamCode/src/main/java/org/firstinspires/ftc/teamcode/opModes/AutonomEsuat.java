package org.firstinspires.ftc.teamcode.opModes;

import com.disnodeteam.dogecv.DogeCV;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import static java.lang.Math.PI;

public class AutonomEsuat extends LinearOpMode {
    private OpenCvCamera webcam;
    private int resWidth = 800, resHeight = 448;
    private Point p1 = new Point(resHeight * 0.55, 0);
    private Point p2 = new Point(resHeight, resWidth);
    private StoneDetectorModified stoneDetectorModified = new StoneDetectorModified(p1, p2);
    private RevBulkData bulkData;
    private ExpansionHubMotor encoderDreapta, encoderSpate, encoderStanga;
    private ExpansionHubEx expansionHub;
    private double forward, right, clockwise, tempforward, tempright;
    double lastCorrectedY = 0, lastCorrectedX = 0, correctedY = 0, correctedX = 0, deltaY, deltaX, sindeltay, sindeltax, cosdeltay, cosdeltax;
    double currentY, currentX, encRot;
    volatile double robotHeading, lastRobotHeading; // sens trigonometric +
    double d = 377, omniLengthMm = 188.49555921538759430775860299677, mmPerTick = omniLengthMm / 4000, rotationCircleLenght = PI * d, tickPerDeg = (rotationCircleLenght / omniLengthMm / 360) * 4000;
    long EncSp, EncSt, EncDr, lastEncDr;
    boolean stop = false;
    volatile boolean encodereCitite = false;
    long timestamp, lastTimestamp, LAST;
    private ExpansionHubMotor motorss, motorsf, motords, motordf;
    double targetAngle = 0, targetDist, calculatedAngle, calcPower;
    double kp = 1, kp2 = 0.35;
    int direction;
    private Thread Loc = new Thread(new Runnable() {
        long encSpVal, encDrVal, encStVal;

        @Override
        public void run() {
            while (!isStopRequested()) {
                bulkData = expansionHub.getBulkInputData();
                encSpVal = bulkData.getMotorCurrentPosition(encoderSpate);
                encDrVal = bulkData.getMotorCurrentPosition(encoderDreapta);
                encStVal = bulkData.getMotorCurrentPosition(encoderStanga);
                EncSp = -encSpVal;
                EncSt = -encStVal;
                EncDr = -encDrVal;

                encRot = ((EncDr - EncSt) / 2.0);
                robotHeading = Math.toRadians(encRot / tickPerDeg) % (2 * PI); //TODO: De vazut daca functioneaza bine asta (ar trebui sa fie de la 0 la 2PI, si sa creasca gradual rotindu-se in sensul acelor de ceas 

                lastCorrectedY = correctedY;
                lastCorrectedX = correctedX;

                correctedY = (EncSt + EncDr) / 2.0;
                correctedX = EncSp + 0.656 * encRot;

                deltaY = correctedY - lastCorrectedY;
                deltaX = correctedX - lastCorrectedX;

                cosdeltay = (Math.cos(robotHeading) * deltaY);
                cosdeltax = (Math.cos((PI / 2) - robotHeading) * deltaX);
                sindeltay = -(Math.sin(robotHeading) * deltaY);
                sindeltax = (Math.sin((PI / 2) - robotHeading) * deltaX);

                currentY = currentY + (cosdeltay + cosdeltax) * mmPerTick;  //TODO: De verificat daca merg OK
                currentX = currentX + (sindeltay + sindeltax) * mmPerTick;
            }
        }
    });

    public void calcule(double X, double Y) {
        boolean jumpDetected = false;
        double dx = X - currentX;
        double dy = Y - currentY;

        if (dx > 0) {
            if (dy > 0) {
                //1
                calculatedAngle = 2*PI - (Math.atan(dx / dy));
            } else {
                //2
                calculatedAngle = 2*PI - (Math.atan(dx / dy) + PI);
            }
        } else {
            if (dy > 0) {
                //4
                calculatedAngle = 2*PI - (Math.atan(dx / dy) + 2 * PI);
            } else {
                //3
                calculatedAngle = 2*PI - (Math.atan(dx / dy) + PI);
            }
        }
        targetAngle = calculatedAngle - robotHeading;
        if(targetAngle > PI){
            targetAngle = 2 * PI - targetAngle;
            direction = -1;
        }
        else{
            direction = 1;
        }
        
        //targetDist = Math.sqrt((X - currentX) * (X - currentX) + (Y - currentY) * (Y - currentY));
        if(direction == -1){
            while(!jumpDetected || calculatedAngle - robotHeading <0){
                if(lastRobotHeading < robotHeading){
                    jumpDetected = true;
                }//
                targetAngle = calculatedAngle - robotHeading;
                calcPower= -(targetAngle * kp); // cu +
                if(calcPower > 1){
                    calcPower = 1;
                }
                power(-calcPower, -calcPower, calcPower, calcPower);
                telemetry.addData("targetAngle", targetAngle);
                telemetry.update();
            }
        }
        else{
            while(targetAngle < 0){
                targetAngle = robotHeading + calculatedAngle;
                calcPower= -(targetAngle * kp); // cu +
                if(calcPower > 1){
                    calcPower = 1;
                }
                power(-calcPower, -calcPower, calcPower, calcPower);
                telemetry.addData("targetAngle", targetAngle);
                telemetry.update();
            }
        }
        power(0,0,0,0);
    }


    private void miscare_fata(double targetY) {
        double powerY;
        while (currentY <= targetY) {
            powerY = kp2 * (targetY - currentY);
            if (powerY > 1) {
                powerY = 1;
            }
            if (powerY < -1) {
                powerY = -1;
            }
            power(powerY, powerY, powerY, powerY);
        }
        power(0, 0, 0, 0);
    }


    @Override
    public void runOpMode() {

        expansionHub = hardwareMap.get(ExpansionHubEx.class, configs.expansionHubOdometrieName);
        motorss = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, configs.ssName);
        motords = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, configs.dsName);
        motorsf = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, configs.sfName);
        motordf = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, configs.dfName);

        encoderDreapta = motordf;
        encoderSpate = motorsf;
        encoderStanga = motorss;

        motordf.setPower(0);
        motords.setPower(0);
        motorsf.setPower(0);
        motorss.setPower(0);

        motordf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motords.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorsf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorss.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motordf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motords.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorsf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorss.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motordf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motords.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorsf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorss.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motords.setDirection(DcMotorSimple.Direction.REVERSE);
        motordf.setDirection(DcMotorSimple.Direction.REVERSE);


        stoneDetectorModified.stonesToFind = 1;
        stoneDetectorModified.useDefaults();
        stoneDetectorModified.filter = new SkystoneDetector(p1, p2);
        stoneDetectorModified.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        webcam.setPipeline(stoneDetectorModified);
        webcam.startStreaming(resWidth, resHeight, OpenCvCameraRotation.SIDEWAYS_LEFT);
        Loc.start();

        //power(-0.3,-0.3,0.3,0.3); // se roteste cu minus

        while (!isStarted()) {
            if (stoneDetectorModified.foundRectangles().get(0).y > 406) {
                telemetry.addData("Skystone Position:", "LEFT");
            } else if (stoneDetectorModified.foundRectangles().get(0).y > 253) {
                telemetry.addData("Skystone Position", "CENTER");
            } else {
                telemetry.addData("Skystone Position", "RIGHT");
            }
            telemetry.addData("EncDr", EncDr);
            telemetry.addData("EncSt", EncSt);
            telemetry.addData("EncSp", EncSp);
            telemetry.addData("X", currentX);
            telemetry.addData("Y", currentY);
            telemetry.addData("atan", calculatedAngle);
            telemetry.addData("Angle ", robotHeading);
            telemetry.addData("targetAngle", targetAngle);
            telemetry.addData("targetDistance", targetDist);
            telemetry.update();
        }
        waitForStart();
        //  calcule(10000,10000);
        miscare_fata(1000);
    }

    private void power(double ds, double df, double ss, double sf) {
        motordf.setPower(df);
        motorss.setPower(ss);
        motorsf.setPower(sf);
        motords.setPower(ds);
    }
}
