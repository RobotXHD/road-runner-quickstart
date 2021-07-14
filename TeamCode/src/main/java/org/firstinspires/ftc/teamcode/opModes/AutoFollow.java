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
public class AutoFollow extends LinearOpMode {
    Hardware_Cam cam = new Hardware_Cam();
    PIDControllerAdevarat pidCam = new PIDControllerAdevarat(0, 0, 0);
    int caz;
    double systemTime;

    @Override
    public void runOpMode() throws InterruptedException {
        cam.Init(hardwareMap);

        pidCam.setSetpoint(100);
        pidCam.setPID(camConfig.kp, camConfig.ki, camConfig.kd);
        pidCam.enable();

        /** detection */
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Ceva: ", cam.skystoneDetectorModified.foundScreenPositions().get(0).x);
            telemetry.addData("Altceva: ", cam.skystoneDetectorModified.foundScreenPositions().get(0).y);
            telemetry.update();
        }

        waitForStart();
    }

    public void asteptare(int milisecunde) {
        systemTime = System.currentTimeMillis();
        while (systemTime + milisecunde > System.currentTimeMillis()) ;
    }
}
