package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
@TeleOp
public class SplineTests extends LinearOpMode {

    @Override

    public void runOpMode() throws InterruptedException {
              SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);
             Hardware_Scissor_V1 r = new Hardware_Scissor_V1();
             r.Init(hardwareMap);
            waitForStart();
            r.startColect();
            if (isStopRequested()) return;
            drive.setPoseEstimate(new Pose2d(-35 * 2.54, 62 * 2.54, Math.toRadians(-90)));
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .setReversed(false)
                            .splineTo(new Pose2d(-62 * 2.54, 24*2.54, Math.toRadians(-135)))
                            .build()
            );
            drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .setReversed(true)
                        .splineTo(new Pose2d(0 * 2.54, 40 * 2.54, Math.toRadians(-180)))
                        .splineTo(new Pose2d(48 * 2.54, 30 * 2.54, Math.toRadians(-270)))
                        .build()
            );
            r.startColectReverse();
            r.prindrePlate();

            sleep(3000);

            drive.followTrajectorySync(
            drive.trajectoryBuilder()
                .setReversed(false)
                .splineTo(new Pose2d(0 * 2.54, 40 * 2.54, Math.toRadians(-180)))

                .build()
            );
        r.desprindrePlate();
        sleep(1000);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .setReversed(false)
                        .splineTo(new Pose2d(-39 * 2.54, 24 * 2.54, Math.toRadians(-135)))
                        .build()
        );
    }
}
