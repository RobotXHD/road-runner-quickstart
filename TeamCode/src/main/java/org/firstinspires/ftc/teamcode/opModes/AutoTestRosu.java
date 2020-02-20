package org.firstinspires.ftc.teamcode.opModes;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;


public class AutoTestRosu extends LinearOpMode {
    Hardware_Cam cam = new Hardware_Cam();
    SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);
    Hardware_Scissor_V1 r = new Hardware_Scissor_V1();
    int caz;

    @Override
    public void runOpMode() {
        cam.Init(hardwareMap);
        r.Init(hardwareMap);

        while (!isStarted()) {
            telemetry.addData("Ceva: ", cam.skystoneDetectorModified.foundScreenPositions().get(0).x);

            if (cam.skystoneDetectorModified.foundScreenPositions().get(0).x >= 156) {
                telemetry.addData("Position", "Left");
                caz = -1;
            } else if (cam.skystoneDetectorModified.foundScreenPositions().get(0).x > 38) {
               telemetry.addData("Position", "Center");
                caz = 0;
            } else {
                telemetry.addData("Position", "Right");
                caz = 1;
            }
            telemetry.update();
        }
        waitForStart();

        drive.setPoseEstimate(new Pose2d(-33.8 * 2.54, 63.14 * 2.54, Math.toRadians(-90)));
        r.startColect();
        //primul cub
        if(caz == 1){
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .setReversed(false)
                            .splineTo(new Pose2d(-62 * 2.54, 24*2.54, Math.toRadians(-135)))
                            .build()
            );
        }
        else if(caz == 0){
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .setReversed(false)
                            .splineTo(new Pose2d(-56 * 2.54, 24*2.54, Math.toRadians(-135)))
                            .build()
            );
        }
        else{
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .setReversed(false)
                            .splineTo(new Pose2d(-46 * 2.54, 24*2.54, Math.toRadians(-135)))
                            .build()
            );
        }

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .setReversed(true)
                        .splineTo(new Pose2d(0 * 2.54, 40 * 2.54, Math.toRadians(-180)))
                        .splineTo(new Pose2d(52 * 2.54, 30 * 2.54, Math.toRadians(-270)))
                        .build()
        );
        r.startColectReverse();
        r.prindrePlate();

      /*  sleep(3000);
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
