package org.firstinspires.ftc.teamcode.opModes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous
public class AutoSkystone extends LinearOpMode {
    Hardware_Cam cam = new Hardware_Cam();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    int caz;
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        cam.Init(hardwareMap);
        while (!cam.isInitFinished){}
        telemetry.addData("I'm already ", "here");
        /** Daca esti in afara runOpMode hardwarewMap e gol; de aici vine nullpointer-ul */
        //SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
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
        }
    }
}