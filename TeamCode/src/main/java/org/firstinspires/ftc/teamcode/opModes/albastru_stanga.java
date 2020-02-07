package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

@TeleOp
public class albastru_stanga extends LinearOpMode {

    @Override

    public void runOpMode() throws InterruptedException {
              SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);
             Hardware_Scissor_V1 r = new Hardware_Scissor_V1();
             r.Init(hardwareMap);
            waitForStart();
            r.startColect();
            if (isStopRequested()) return;
            // pozitie plecare
            drive.setPoseEstimate(new Pose2d(-33.8 * 2.54, 63.14 * 2.54, Math.toRadians(-90)));
            //primul cub
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .setReversed(false)
                            .splineTo(new Pose2d(-46 * 2.54, 24*2.54, Math.toRadians(-135)))
                            .build()
            );
            //mers la placa
            drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .setReversed(true)
                        .splineTo(new Pose2d(0 * 2.54, 40 * 2.54, Math.toRadians(-180)))
                        .splineTo(new Pose2d(52 * 2.54, 30 * 2.54, Math.toRadians(-270)))
                        .build()
            );
            r.startColectReverse();
            r.prindrePlate();

            sleep(3000);
            // tras placa in centru
            drive.followTrajectorySync(
            drive.trajectoryBuilder()
                .setReversed(false)
                .splineTo(new Pose2d(0 * 2.54, 40 * 2.54, Math.toRadians(-130)))

                .build()
            );
        r.desprindrePlate();
        r.startColect();
        sleep(600);
            //cub 2
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .setReversed(false)
                        .splineTo(new Pose2d(-27* 2.54, 24 * 2.54, Math.toRadians(-135)))
                        .build()
        );
        //pus placa in colt
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .setReversed(true)
                        .splineTo(new Pose2d(0 * 2.54, 40 * 2.54, Math.toRadians(180)))
                        .splineTo(new Pose2d(50 * 2.54, 50 * 2.54, Math.toRadians(180)))
                        .build()
        );
        //parcare robot sub bridge
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .setReversed(false)
                        .splineTo(new Pose2d(0 * 2.54, 40 * 2.54, Math.toRadians(-180)))

                        .build()
        );
        r.stop = true;
    }
}
