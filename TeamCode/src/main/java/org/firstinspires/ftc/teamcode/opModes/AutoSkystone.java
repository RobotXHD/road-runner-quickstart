package org.firstinspires.ftc.teamcode.opModes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.opencv.core.Mat;


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
        Trajectory trajectory = new TrajectoryBuilder(new Pose2d(0,0,0), DriveConstants.BASE_CONSTRAINTS)
                .lineTo(new Vector2d(100,0), new LinearInterpolator(Math.toRadians(0), Math.toRadians(90)))
                .build();
        drive.followTrajectorySync(trajectory);
    }
}