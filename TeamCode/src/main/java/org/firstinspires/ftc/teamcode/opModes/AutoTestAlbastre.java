package org.firstinspires.ftc.teamcode.opModes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;


@Autonomous
public class AutoTestAlbastre extends LinearOpMode {
    Hardware_Cam cam = new Hardware_Cam();
    Hardware_Scissor_V1 sisteme = new Hardware_Scissor_V1();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    int caz;
    SampleMecanumDriveBase drive;
    @Override
    public void runOpMode() {
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        cam.Init(hardwareMap);
        sisteme.Init(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-38.52 * 2.54, 62.18 * 2.54, Math.toRadians(-90)));
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        while (!isStarted()) {
            telemetry.addData("Ceva: ", cam.skystoneDetectorModified.foundScreenPositions().get(0).x);

            if (cam.skystoneDetectorModified.foundScreenPositions().get(0).x >= 156) {
                telemetry.addData("Position", "Left");
                caz = -1;
            } else if (cam.skystoneDetectorModified.foundScreenPositions().get(0).x > 38) {
                telemetry.addData("Position", "Right");
                caz = 1;
            } else {
                telemetry.addData("Position", "Center");
                caz = 0;

            }
            telemetry.update();
        }
        sisteme.stop = false;
        waitForStart();
        sisteme.startColect();
        //primul cub
        if (caz == 1) {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .setReversed(false)
                            .splineTo(new Pose2d(-57 * 2.54, 31 * 2.54, Math.toRadians(-120)))
                            .build()
            );
        } else if (caz == 0) {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .setReversed(false)
                            .splineTo(new Pose2d(-51.5 * 2.54, 31 * 2.54, Math.toRadians(-120)))
                            .build()
            );
        } else {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .setReversed(false)
                            .splineTo(new Pose2d(-35 * 2.54, 36 * 2.54, Math.toRadians(-135)))
                            .splineTo(new Pose2d(-41 * 2.54, 31 * 2.54, Math.toRadians(-135)))
                            .build()
            );

        }
        sisteme.servoPlatformaDr.setPosition(0.3);
        sisteme.servoPlatformaSt.setPosition(0.7);
        // mers la placa
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .setReversed(true)
                        .splineTo(new Pose2d(10 * 2.54, 38 * 2.54, Math.toRadians(-180)))
                        //  .lineTo(new Vector2d(48 * 2.54,30 * 2.54), new LinearInterpolator(Math.toRadians(90), Math.toRadians(0))
                        /**sisteme.aruncaCuburi();
                         prindere placa**/
                        .splineTo(new Pose2d(42 * 2.54, 30 * 2.54, Math.toRadians(-270)))
                        .build()
        );
        sisteme.prindrePlate();
        sleep(1000);
        sisteme.servoClamp.setPosition(configs.pozitie_servoClamp_prindere);
        sisteme.stopColect();
        sleep(1000);
        sisteme.aruncaCuburi();
        sleep(1000);




        //trasa placa in centru
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .setReversed(false)
                        //.splineTo(new Pose2d(29 * 2.54, 39 * 2.54, Math.toRadians(-180)))
                   //     .strafeTo(new Vector2d(20 * 2.54,31 * 2.54))
                        .splineTo(new Pose2d(10 * 2.54, 38 * 2.54, Math.toRadians(-180)))
                        .build()
        );
       /* sisteme.desprindrePlate();
        sleep(1000);
        //mers cub 2
        sisteme.startColect();
        if(caz == 1){

        }
        else if(caz == 0){
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .setReversed(false)
                            .splineTo(new Pose2d(-26 * 2.54, 28 * 2.54, Math.toRadians(-120)))
                            .build()
            );
        }
        else{
        }

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .setReversed(true)
                        .splineTo(new Pose2d(15 * 2.54, 38 * 2.54, Math.toRadians(-180)))
                        .build()
        );
        sisteme.servoClamp.setPosition(configs.pozitie_servoClamp_prindere);
        sleep(1000);
        sisteme.stopColect();
        sleep(1000);
        sisteme.aruncaCuburi();
        sleep(1000);
        sisteme.stop = true;

      //  sisteme.aruncaCuburi();

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
    }

    public void gotoCub(){
        boolean isCollected = false;
        double camPower = 0.0, power = 0.3;
        while(!isCollected){
            isCollected = sisteme.touchGheara.isPressed();
            camPower = sisteme.pidCam.performPID(cam.skystoneDetectorModified.foundRectangles().get(0).x + (cam.skystoneDetectorModified.foundRectangles().get(0).width) / 2);
            drive.setMotorPowers(power-camPower, power-camPower, power+camPower, power+camPower);
        }
    }
}
