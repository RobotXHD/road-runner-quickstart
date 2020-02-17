package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

@Autonomous
public class AutoBlue_v1_Strafe extends LinearOpMode {
    Hardware_Cam cam = new Hardware_Cam();
    PIDControllerAdevarat pidCam = new PIDControllerAdevarat(0, 0, 0);
    boolean isCollected = false;
    int caz;
    SampleMecanumDriveREVOptimized drive;

    boolean isScissorExtended = false, isCubeThrown = false, isCubeCaught = false;

    @Override
    public void runOpMode() throws InterruptedException {
        cam.Init(hardwareMap);
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        drive.Init(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-38.52 * 2.54, 62.18 * 2.54, Math.toRadians(-90)));
        pidCam.setSetpoint(100);
        pidCam.setPID(camConfig.kp, camConfig.ki, camConfig.kd);
        pidCam.enable();
        drive.servoClamp.setPosition(configs.pozitie_servoClamp_desprindere);
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

        waitForStart();
        drive.Colect(-0.7);
        //primul cub
        if (caz == 1) {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .setReversed(false)
                            .strafeTo(new Vector2d(-53 * 2.54, 41 * 2.54))
                            .lineTo(new Vector2d(-53 * 2.54, 36 * 2.54), new LinearInterpolator(Math.toRadians(-90), Math.toRadians(-45)))
                            .lineTo(new Vector2d(-53 * 2.54, 31 * 2.54), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                            .lineTo(new Vector2d(-58 * 2.54, 26 * 2.54), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                            .build()
            );
        } else if (caz == 0) {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .setReversed(false)
                            .strafeTo(new Vector2d(-45.5 * 2.54, 55.7 * 2.54))
                            .lineTo(new Vector2d(-45.5 * 2.54, 36 * 2.54), new LinearInterpolator(Math.toRadians(-90), Math.toRadians(-45)))
                            .lineTo(new Vector2d(-45.5 * 2.54, 31 * 2.54), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                            .lineTo(new Vector2d(-50.5 * 2.54, 26 * 2.54), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                            .build()
            );
        } else {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .setReversed(false)
                            .strafeTo(new Vector2d(-39 * 2.54, 60.66 * 2.54))
                            .lineTo(new Vector2d(-39 * 2.54, 36 * 2.54), new LinearInterpolator(Math.toRadians(-90), Math.toRadians(-45)))
                            .lineTo(new Vector2d(-39 * 2.54, 31 * 2.54), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                            .lineTo(new Vector2d(-44 * 2.54, 26 * 2.54), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                            .build()
            );
        }

        drive.desprindrePlate();
        drive.Colect(-1);
        sleep(500);
        drive.servoClamp.setPosition(configs.pozitie_servoClamp_prindere);
        /**mers la placa*/
        drive.followTrajectory(
                drive.trajectoryBuilder()
                        .setReversed(true)
                        .splineTo(new Pose2d(10 * 2.54,38 * 2.54 ,Math.toRadians(-180)))
                        .splineTo(new Pose2d(15 * 2.54,38 * 2.54, Math.toRadians(-180)))
                        .splineTo(new Pose2d(43.5 * 2.54, 38 * 2.54, Math.toRadians(-270)))
                        .build()
        );

        while(drive.isBusy()){
            drive.update();
            drive.updatePoseEstimate();
            if(drive.getPoseEstimate().getX() > -38 && !isCubeCaught){
                drive.servoClamp.setPosition(configs.pozitie_servoClamp_prindere);
                drive.stopColect();
                isCubeCaught = true;
            }
            if(drive.getPoseEstimate().getX() > 15 && !isScissorExtended){
                drive.extensieScissor();
                isScissorExtended = true;
            }
        }

        Trajectory trajectory = new TrajectoryBuilder(drive.getPoseEstimate(), new DriveConstraints(20,150,0,200,360,0))
                .setReversed(true)
                .splineTo(new Pose2d(43.5 * 2.54, 32 * 2.54, Math.toRadians(-270)))
                .strafeTo(new Vector2d(43.5 * 2.54, 29 * 2.54))
                .build();

        drive.followTrajectorySync(
                trajectory
        );

        drive.prindrePlate();
        drive.servoClamp.setPosition(configs.pozitie_servoClamp_desprindere);
        sleep(500);
        isCubeCaught = false;
        isScissorExtended = false;

        //drive.startColectReverse();
        /*
        currentPos = drive.getPoseEstimate();

        telemetry.addData("Pos", drive.getPoseEstimate());
        drive.aruncaCuburi();
        drive.update();
        telemetry.addData("PosNou", drive.getPoseEstimate());
        telemetry.update();
        drive.setPoseEstimate(currentPos);
        sleep(10000);
        //drive.startColectReverse();

        /**tras placa

        drive.turnSync(Math.toRadians(10));

        drive.followTrajectory(
                drive.trajectoryBuilder()
                        .setReversed(false)
                        .splineTo(new Pose2d(32*2.54,45*2.54,Math.toRadians(-180)))
                        .splineTo(new Pose2d(20*2.54,40*2.54, Math.toRadians(-180)))
                        .build()
        );

        drive.homeScissor();

        while(drive.isBusy()){
            drive.update();
            drive.updatePoseEstimate();
        }

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .setReversed(false)
                        .splineTo(new Pose2d(15*2.54,38*2.54, Math.toRadians(-180)))
                        .build()
        );
        drive.startColect();
        //cub 2
        drive.desprindrePlate();
        sleep(500);


        if(caz == 1){
            //inca nu
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .setReversed(false)
                            .splineTo(new Pose2d(-36.5* 2.54, 26 * 2.54, Math.toRadians(-130)))
                            .build()
            );
        }
        else if(caz == 0){
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .setReversed(false)
                            .splineTo(new Pose2d(-8 * 2.54, 38 * 2.54, Math.toRadians(-180)))
                            .splineTo(new Pose2d(-27.5 * 2.54, 26 * 2.54, Math.toRadians(-125)))
                            .build()
            );
        }
        else{
            //this nu
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .setReversed(false)
                            .splineTo(new Pose2d(-19.5 * 2.54, 26 * 2.54, Math.toRadians(-130)))
                            .build()
            );
        }
        drive.Colect(-1);
        sleep(500);
        drive.servoClamp.setPosition(configs.pozitie_servoClamp_prindere);

        drive.followTrajectory(
                drive.trajectoryBuilder()
                        .setReversed(true)
                        .splineTo(new Pose2d(-12.5 * 2.54, 38 * 2.54, Math.toRadians(-180)))
                        //.splineTo(new Pose2d(40 * 2.54, 38 * 2.54, Math.toRadians(-180)))
                        .build()
        );
        while(drive.isBusy()){
            drive.update();
            drive.updatePoseEstimate();
            if(drive.getPoseEstimate().getX() > -10 && !isCubeCaught){
                drive.servoClamp.setPosition(configs.pozitie_servoClamp_prindere);
                drive.stopColect();
                isCubeCaught = true;
            }
            if(drive.getPoseEstimate().getX() > 15 && !isScissorExtended){
                drive.extensieScissor();
                isScissorExtended = true;
            }
        }
        drive.servoClamp.setPosition(configs.pozitie_servoClamp_desprindere);
        drive.followTrajectory(
                drive.trajectoryBuilder()
                        .setReversed(false)
                        .splineTo(new Pose2d(0,38 * 2.54, Math.toRadians(-180)))
                        .build()
        );
        drive.homeScissor();
        while(!drive.isBusy()){
            drive.update();
            drive.updatePoseEstimate();
        }
        drive.setMotorPowers(0,0,0,0);
        sleep(1000);
        /*
        drive.turnSync(Math.toRadians(-100));
        drive.startColectReverse();
        sleep(1000);
        drive.stopColect();
        drive.turnSync(Math.toRadians(100));
        /*
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(-20 * 2.54, 38 * 2.54, Math.toRadians(-130)))
                        .build()
        );
        colectCub();
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .setReversed(true)
                        .splineTo(new Pose2d(-15 * 2.54, 38 * 2.54, Math.toRadians(-180)))
                        .splineTo(new Pose2d(12 * 2.54, 38 * 2.54, Math.toRadians(-180)))
                        .build()
        );
        drive.turnSync(Math.toRadians(-100));
        drive.startColectReverse();
        sleep(1000);
        drive.stopColect();
        drive.turnSync(Math.toRadians(100));
         */
        drive.stop = true;
    }

    public void colectCub() {
        double power = 0.5, camPower;
        drive.startColect();
        cam.startDetection(new StoneDetector(480, 640));
        sleep(4000);
        while (!isCollected) {
            drive.update();
            drive.updatePoseEstimate();
            telemetry.addData("Cube", cam.skystoneDetectorModified.foundRectangles().get(0));
            telemetry.update();
            if (cam.skystoneDetectorModified.foundRectangles().get(0).width < 50) {
                power = 0.4;
            } else {
                power = 0.5;
            }
            isCollected = drive.touchGheara.isPressed();
            pidCam.setPID(camConfig.kp, camConfig.ki, camConfig.kd);
            camPower = pidCam.performPID(cam.skystoneDetectorModified.foundRectangles().get(0).x + (cam.skystoneDetectorModified.foundRectangles().get(0).width) / 2);
            drive.setMotorPowers(power - camPower, power - camPower, power + camPower, power + camPower);
        }

        drive.stopColect();
        drive.setMotorPowers(0, 0, 0, 0);
    }
}
