package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;


@Autonomous
public class AutoSputnik extends LinearOpMode {
    Hardware_Cam cam = new Hardware_Cam();
    PIDControllerAdevarat pidCam = new PIDControllerAdevarat(0, 0, 0);
    boolean isCollected = false;
    int caz;
    double systemTime, startTime;
    SampleMecanumDriveREVOptimized drive;

    public void runOpMode() throws InterruptedException {
        DriveConstants.BASE_CONSTRAINTS = new DriveConstraints(120, 150, 0, 360, 360, 0);
        cam.Init(hardwareMap);
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        drive.Init(hardwareMap);
        drive.setPoseEstimate(new Pose2d(96, 62.18 * 2.54, Math.toRadians(90)));
        drive.update();
        drive.updatePoseEstimate();

        drive.NOTDUCK = false;
        drive.servoClamp.setPosition(configs.pozitie_servoClamp_desprindere);

        waitForStart();
        Trajectory trajectory = new TrajectoryBuilder(drive.getPoseEstimate(), new DriveConstraints(25, 150, 0, 200, 360, 0))
                .setReversed(true)
                .splineTo(new Pose2d(43.5 * 2.54, 32 * 2.54, Math.toRadians(-270)))
                .strafeTo(new Vector2d(43.5 * 2.54, 29 * 2.54))
                .build();

        drive.desprindrePlate();

        drive.followTrajectorySync(
                trajectory
        );

        drive.prindrePlate();
        sleep(500);

        /**tras placa */


        drive.turnSync(Math.toRadians(10));

        drive.followTrajectory(
                drive.trajectoryBuilder()
                        .setReversed(false)
                        .splineTo(new Pose2d(32 * 2.54, 45 * 2.54, Math.toRadians(-200)))
                        .splineTo(new Pose2d(20 * 2.54, 38 * 2.54, Math.toRadians(-180)))
                        .build()
        );

        while (drive.isBusy()) {
            drive.update();
            drive.updatePoseEstimate();
        }

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .setReversed(false)
                        .strafeTo(new Vector2d(0, 62.18))
                        .build()
        );
        drive.protectiePlate();
        drive.stop = true;
    }
}
