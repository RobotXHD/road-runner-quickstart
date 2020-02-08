package org.firstinspires.ftc.teamcode.opModes;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;


@Autonomous
public class AutoSkystone extends LinearOpMode {
    Hardware_Cam cam = new Hardware_Cam();
    SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);
    Hardware_Scissor_V1 r = new Hardware_Scissor_V1();
    int caz;
    @Override
    public void runOpMode() {
        cam.Init(hardwareMap);
        r.Init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Ceva: ", cam.stoneDetectorModified.foundScreenPositions().get(0).y);

            if (cam.stoneDetectorModified.foundScreenPositions().get(0).y >= 140) {
                telemetry.addData("Position", "CENTRE");
                caz = 0;
            } else if (cam.stoneDetectorModified.foundScreenPositions().get(0).y > 70) {
                telemetry.addData("Position", "RIGHT");
                caz = 1;
            } else {
                telemetry.addData("Position", "LEFT");
                caz = -1;

            }
            telemetry.update();
        }
    }
}