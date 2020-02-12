package org.firstinspires.ftc.teamcode.opModes;


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

import java.util.Vector;


@Autonomous
public class AutoTestAlbastre extends LinearOpMode {
    Hardware_Cam cam = new Hardware_Cam();
    Hardware_Scissor_V1 r = new Hardware_Scissor_V1();
    int caz;

    @Override
    public void runOpMode() {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        cam.Init(hardwareMap);
        r.Init(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-38.52 * 2.54, 62.18 * 2.54, Math.toRadians(-90)));
        while (!isStarted()) {
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
        waitForStart();
        r.startColect();
        //primul cub
        if(caz == 1){
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .setReversed(false)
                            .splineTo(new Pose2d(-57 * 2.54, 31*2.54, Math.toRadians(-135)))
                            .build()
            );
        }
        else if(caz == 0){
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .setReversed(false)
                            .splineTo(new Pose2d(-49 * 2.54, 31*2.54, Math.toRadians(-135)))
                            .build()
            );
        }
        else{
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .setReversed(false)
                            .splineTo(new Pose2d(-35 * 2.54, 36*2.54, Math.toRadians(-135)))
                            .splineTo(new Pose2d(-41*2.54, 31*2.54, Math.toRadians(-135)))
                            .build()
            );

        }

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .setReversed(true)
                        .splineTo(new Pose2d(10 * 2.54,38 * 2.54, Math.toRadians(-180)))
                        .lineTo(new Vector2d(61 * 2.54,38 * 2.54), new LinearInterpolator(Math.toRadians(90), Math.toRadians(0)))
                        .lineTo(new Vector2d(61 * 2.54,31 * 2.54))
                        .build()
        );
        r.prindrePlate();
        r.startColectReverse();
        sleep(3000);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .setReversed(false)
                        .splineTo(new Pose2d(20 * 2.54,60 * 2.54, Math.toRadians(-180)))
                        .build()
        );
        r.stopColect();
        r.desprindrePlate();
        sleep(1000);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeTo(new Vector2d(20 * 2.54, 38 * 2.54))
                        .build()
        );

        r.startColect();
        if(caz == 1){

        }
        else if(caz == 0){
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .setReversed(false)
                            .splineTo(new Pose2d(-28 * 2.54, 34 * 2.54, Math.toRadians(-135)))
                            .build()
            );
        }
        else{

        }

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .setReversed(true)
                        .splineTo(new Pose2d(20 * 2.54, 38 * 2.54, Math.toRadians(-180)))
                        .build()
        );

        /*
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .setReversed(true)
                        .splineTo(new Pose2d(0 * 2.54, 40 * 2.54, Math.toRadians(-180)))
                        .splineTo(new Pose2d(41 * 2.54, 38   * 2.54, Math.toRadians(-270)))
                        .build()
        );
        r.startColectReverse();



        sleep(3000);
        /*
        // tras placa in centru
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .setReversed(false)
                        .splineTo(new Pose2d(0 * 2.54, 40 * 2.54, Math.toRadians(-130)))
                        .build()
        );
*/
        r.stop = true;
    }
}
