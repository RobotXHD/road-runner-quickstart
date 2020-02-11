package org.firstinspires.ftc.teamcode.opModes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;


@Autonomous
public class AutoSkystone extends LinearOpMode {
    Hardware_Cam cam = new Hardware_Cam();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();
    int caz;
    @Override
    public void runOpMode() {
        cam.Init(hardwareMap);
        while (!cam.isInitFinished){}
        packet.put("I'm already ", "here");
        dashboard.sendTelemetryPacket(packet);
        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap); /** Daca esti in afara runOpMode hardwarewMap e gol; de aici vine nullpointer-ul */
        waitForStart();
        drive.setPoseEstimate(new Pose2d(-33.8 * 2.54, 63.14 * 2.54, Math.toRadians(-90)));
        while (opModeIsActive()) {
            telemetry.addData("Ceva: ", cam.stoneDetectorModified.foundScreenPositions().get(0).x);

            if (cam.stoneDetectorModified.foundScreenPositions().get(0).x >= 156) {
                telemetry.addData("Position", "Left");
                caz = -1;
            } else if (cam.stoneDetectorModified.foundScreenPositions().get(0).x > 38) {
                telemetry.addData("Position", "Right");
                caz = 1;
            } else {
                telemetry.addData("Position", "Center");
                caz = 0;

            }
            telemetry.update();
        }
        if(caz==1){
            //primul cub, caz 1
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .setReversed(false)
                            .splineTo(new Pose2d(-46 * 2.54, 24*2.54, Math.toRadians(-135)))
                            .build()
            );
            //tras placa, caz 1
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .setReversed(true)
                            .splineTo(new Pose2d(0 * 2.54, 40 * 2.54, Math.toRadians(-180)))
                            .splineTo(new Pose2d(52 * 2.54, 30 * 2.54, Math.toRadians(-270)))
                            .build()
            );

        }
    }
}