package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

@Autonomous
public class AutoBlue_v1_part2 extends LinearOpMode {
    Hardware_Cam cam = new Hardware_Cam();
    PIDControllerAdevarat pidCam = new PIDControllerAdevarat(0,0,0);
    boolean isCollected = false;
    int caz;
    Pose2d pose2d;
    SampleMecanumDriveREVOptimized drive;
    @Override
    public void runOpMode() throws InterruptedException {
        cam.Init(hardwareMap);
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        drive.Init(hardwareMap);
        drive.setPoseEstimate(new Pose2d(43 * 2.54, 0 * 2.54, Math.toRadians(-270)));
        pidCam.setSetpoint(100);
        pidCam.setPID(camConfig.kp, camConfig.ki, camConfig.kd);
        pidCam.enable();
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
        pose2d = drive.getPoseEstimate();
        telemetry.addData("INITPOZ:", drive.getPoseEstimate());
        drive.prindrePlate();
        //drive.startColectReverse();
        drive.servoClamp.setPosition(configs.pozitie_servoClamp_prindere);
        sleep(2000);
        drive.stopColect();
        drive.aruncaCuburi();
        drive.updatePoseEstimate();
        drive.update();
        drive.setPoseEstimate(new Pose2d(pose2d.getX(), pose2d.getY(), Math.toRadians(-270)));
        drive.updatePoseEstimate();
        drive.update();
        sleep(1000);
        telemetry.addData("POZ:", drive.getPoseEstimate());
        telemetry.update();
        sleep(5000);
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

        /**tras placa*/
        drive.turnSync(Math.toRadians(10));

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .setReversed(false)
                        .splineTo(new Pose2d(32*2.54,45*2.54,Math.toRadians(-180)))
                        .splineTo(new Pose2d(20*2.54,40*2.54, Math.toRadians(-180)))
                        .build()
        );
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .setReversed(false)
                        .splineTo(new Pose2d(15*2.54,38*2.54, Math.toRadians(-180)))
                        .build()
        );
        drive.startColect();
        //cub 2
        drive.desprindrePlate();
        sleep(1000);

        if(caz == 1){
            //inca nu
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .setReversed(false)
                            .splineTo(new Pose2d(- 27* 2.54, 28 * 2.54, Math.toRadians(-135)))
                            .build()
            );
        }
        else if(caz == 0){
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .setReversed(false)
                            .splineTo(new Pose2d(-12.5 * 2.54, 38 * 2.54, Math.toRadians(-180)))
                            .splineTo(new Pose2d(-27.5 * 2.54, 28 * 2.54, Math.toRadians(-130)))
                            .build()
            );
        }
        else{
            //this nu
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .setReversed(false)
                            .splineTo(new Pose2d(-27 * 2.54, 28 * 2.54, Math.toRadians(-135)))
                            .build()
            );
        }

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .setReversed(true)
                        .splineTo(new Pose2d(-12.5 * 2.54, 38 * 2.54, Math.toRadians(-180)))
                        .splineTo(new Pose2d(12 * 2.54, 38 * 2.54, Math.toRadians(-180)))
                        .build()
        );
        drive.turnSync(Math.toRadians(-100));
        drive.startColectReverse();
        sleep(1000);
        drive.stopColect();
        drive.turnSync(Math.toRadians(100));
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(-15 * 2.54, 38 * 2.54, Math.toRadians(-90)))
                        .build()
        );
        colectCub();
        drive.stop = true;
    }

    public void colectCub(){
        double power = 0.4, camPower;
        drive.startColect();
        cam.startDetection(new StoneDetector(480, 640));
        sleep(1000);
        while (!isCollected) {
            telemetry.addData("Cube", cam.skystoneDetectorModified.foundRectangles().get(0));
            telemetry.update();
            if(cam.skystoneDetectorModified.foundRectangles().get(0).width < 50){
                power = 0.2;
            }
            else{
                power = 0.4;
            }
            isCollected = drive.touchGheara.isPressed();
            pidCam.setPID(camConfig.kp, camConfig.ki, camConfig.kd);
            camPower = pidCam.performPID(cam.skystoneDetectorModified.foundRectangles().get(0).x + (cam.skystoneDetectorModified.foundRectangles().get(0).width) / 2);
            drive.setMotorPowers(power-camPower, power-camPower, power+camPower, power+camPower);
        }

        drive.stopColect();
        drive.setMotorPowers(0,0,0,0);
    }
}
