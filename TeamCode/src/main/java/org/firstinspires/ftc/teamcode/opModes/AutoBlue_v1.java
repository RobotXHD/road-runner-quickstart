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
public class AutoBlue_v1 extends LinearOpMode {
    Hardware_Cam cam = new Hardware_Cam();
    PIDControllerAdevarat pidCam = new PIDControllerAdevarat(0, 0, 0);
    boolean isCollected = false;
    int caz;
    double systemTime, startTime;
    SampleMecanumDriveREVOptimized drive;

    boolean isScissorExtended = false, isCubeThrown = false, isCubeCaught = false, isFoundationReleased = false, changePipeline = false, isCrazy = false;

    @Override
    public void runOpMode() throws InterruptedException {
        DriveConstants.BASE_CONSTRAINTS = new DriveConstraints(120, 150, 0, 360, 360, 0);
        cam.Init(hardwareMap);
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        drive.Init(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-38.52 * 2.54, 62.18 * 2.54, Math.toRadians(-90)));
        drive.update();
        drive.updatePoseEstimate();

        pidCam.setSetpoint(100);
        pidCam.setPID(camConfig.kp, camConfig.ki, camConfig.kd);
        pidCam.enable();

        drive.servoClamp.setPosition(configs.pozitie_servoClamp_desprindere);
/*

        Trajectory left1 = new TrajectoryBuilder(new Pose2d(-38.52 * 2.54, 62.18 * 2.54, Math.toRadians(-90)), DriveConstants.BASE_CONSTRAINTS)
                .setReversed(false)
                .strafeTo(new Vector2d(-39 * 2.54, 60.66 * 2.54))
                .lineTo(new Vector2d(-39 * 2.54, 36 * 2.54), new LinearInterpolator(Math.toRadians(-90), Math.toRadians(-45)))
                .lineTo(new Vector2d(-39 * 2.54, 34 * 2.54), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                .lineTo(new Vector2d(-47 * 2.54, 26 * 2.54), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                .build();
        Trajectory center1 = new TrajectoryBuilder(new Pose2d(-38.52 * 2.54, 62.18 * 2.54, Math.toRadians(-90)), DriveConstants.BASE_CONSTRAINTS)
                .setReversed(false)
                .strafeTo(new Vector2d(-45.5 * 2.54, 55.7 * 2.54))
                .lineTo(new Vector2d(-45.5 * 2.54, 36 * 2.54), new LinearInterpolator(Math.toRadians(-90), Math.toRadians(-45)))
                .lineTo(new Vector2d(-45.5 * 2.54, 34 * 2.54), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                .lineTo(new Vector2d(-53.5 * 2.54, 26 * 2.54), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                .build();
        Trajectory right1 = new TrajectoryBuilder(new Pose2d(-38.52 * 2.54, 62.18 * 2.54, Math.toRadians(-90)), DriveConstants.BASE_CONSTRAINTS)
                .setReversed(false)
                .strafeTo(new Vector2d(-53 * 2.54, 41 * 2.54))
                .lineTo(new Vector2d(-53 * 2.54, 36 * 2.54), new LinearInterpolator(Math.toRadians(-90), Math.toRadians(-45)))
                .lineTo(new Vector2d(-53 * 2.54, 34 * 2.54), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                .lineTo(new Vector2d(-61 * 2.54, 26 * 2.54), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                .build();
*/
        /** detection */
        inFlightPipelineChange.start();
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Ceva: ", cam.skystoneDetectorModified.foundScreenPositions().get(0).x);
            if (cam.skystoneDetectorModified.foundScreenPositions().get(0).x >= 255) {
                telemetry.addData("Position", "Left");
                caz = -1;
            } else if (cam.skystoneDetectorModified.foundScreenPositions().get(0).x > 95) {
                telemetry.addData("Position", "Right");
                caz = 1;
            } else {
                telemetry.addData("Position", "Center");
                caz = 0;
            }
            telemetry.update();
        }

        waitForStart();
        startTime = System.currentTimeMillis();
        /** primul cub */
        drive.startColect();
        if (caz == 1) {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .setReversed(false)
                            .strafeTo(new Vector2d(-53 * 2.54, 41 * 2.54))
                            .lineTo(new Vector2d(-53 * 2.54, 36 * 2.54), new LinearInterpolator(Math.toRadians(-90), Math.toRadians(-45)))
                            .lineTo(new Vector2d(-53 * 2.54, 34 * 2.54), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                            .lineTo(new Vector2d(-61 * 2.54, 26 * 2.54), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                            .build()
            );
        } else if (caz == 0) {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .setReversed(false)
                            .strafeTo(new Vector2d(-45.5 * 2.54, 55.7 * 2.54))
                            .lineTo(new Vector2d(-45.5 * 2.54, 36 * 2.54), new LinearInterpolator(Math.toRadians(-90), Math.toRadians(-45)))
                            .lineTo(new Vector2d(-45.5 * 2.54, 34 * 2.54), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                            .lineTo(new Vector2d(-53.5 * 2.54, 26 * 2.54), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                            .build()
            );
        } else {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .setReversed(false)
                            .strafeTo(new Vector2d(-39 * 2.54, 60.66 * 2.54))
                            .lineTo(new Vector2d(-39 * 2.54, 36 * 2.54), new LinearInterpolator(Math.toRadians(-90), Math.toRadians(-45)))
                            .lineTo(new Vector2d(-39 * 2.54, 34 * 2.54), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                            .lineTo(new Vector2d(-47 * 2.54, 26 * 2.54), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                            .build()
            );
        }
        changePipeline = true;
        drive.Colect(0.5);
        asteptare(70);
        drive.Colect(-1);
        drive.setMotorPowers(0.2, 0.2, 0.2, 0.2);
        systemTime = System.currentTimeMillis();
        while (!drive.touchGheara.isPressed() && opModeIsActive() && (systemTime + 500 > System.currentTimeMillis())) {
        }
        drive.setMotorPowers(0, 0, 0, 0);
        drive.servoClamp.setPosition(configs.pozitie_servoClamp_prindere);
        /**mers la placa cu ridicare de scissor */
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .setReversed(true)
                        .splineTo(new Pose2d(10 * 2.54, 38 * 2.54, Math.toRadians(-180)))
                        .splineTo(new Pose2d(15 * 2.54, 38 * 2.54, Math.toRadians(-180)))
                        .splineTo(new Pose2d(43.5 * 2.54, 38 * 2.54, Math.toRadians(-270)))
                        .build()
        );

        drive.dubiousExtensie();
        Trajectory trajectory = new TrajectoryBuilder(drive.getPoseEstimate(), new DriveConstraints(25, 150, 0, 200, 360, 0))
                .setReversed(true)
                .splineTo(new Pose2d(43.5 * 2.54, 32 * 2.54, Math.toRadians(-270)))
                .strafeTo(new Vector2d(43.5 * 2.54, 29 * 2.54))
                .build();

        drive.followTrajectory(
                trajectory
        );
        drive.desprindrePlate();
        while (drive.isBusy()) {
            drive.update();
            drive.updatePoseEstimate();
            if (drive.getPoseEstimate().getY() < 31.5) {
                drive.servoClamp.setPosition(configs.pozitie_servoClamp_desprindere);
            }
        }
        drive.servoClamp.setPosition(configs.pozitie_servoPlatformaSt_desprindere);
        drive.prindrePlate();
        asteptare(500);
        isCubeCaught = false;
        isScissorExtended = false;

        /**tras placa */


        drive.turnSync(Math.toRadians(10));

        drive.followTrajectory(
                drive.trajectoryBuilder()
                        .setReversed(false)
                        .splineTo(new Pose2d(32 * 2.54, 45 * 2.54, Math.toRadians(-200)))
                        .splineTo(new Pose2d(20 * 2.54, 38 * 2.54, Math.toRadians(-180)))
                        .build()
        );

        drive.dubiousHome();

        while (drive.isBusy()) {
            drive.update();
            drive.updatePoseEstimate();
        }
        drive.protectiePlate();
        drive.startColect();
        /** cub 2 */
        if (caz == 1) {
            drive.followTrajectory(
                    drive.trajectoryBuilder()
                            .setReversed(false)
                            .splineTo(new Pose2d(15 * 2.54, 38 * 2.54, Math.toRadians(-180)))
                            .splineTo(new Pose2d(-28 * 2.54, 37 * 2.54, Math.toRadians(-135)))
                            .lineTo(new Vector2d(-28 * 2.54, 31 * 2.54), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                            .lineTo(new Vector2d(-34 * 2.54, 26 * 2.54), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                            .build()
            );
        } else if (caz == 0) {
            drive.followTrajectory(
                    drive.trajectoryBuilder()
                            .setReversed(false)
                            .splineTo(new Pose2d(15 * 2.54, 38 * 2.54, Math.toRadians(-180)))
                            .splineTo(new Pose2d(-21 * 2.54, 37 * 2.54, Math.toRadians(-135)))
                            .lineTo(new Vector2d(-21 * 2.54, 31 * 2.54), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                            .lineTo(new Vector2d(-26 * 2.54, 26 * 2.54), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                            .build()
            );
        } else {
            drive.followTrajectory(
                    drive.trajectoryBuilder()
                            .setReversed(false)
                            .splineTo(new Pose2d(15 * 2.54, 38 * 2.54, Math.toRadians(-180)))
                            .splineTo(new Pose2d(-13 * 2.54, 37 * 2.54, Math.toRadians(-135)))
                            .lineTo(new Vector2d(-13 * 2.54, 31 * 2.54), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                            .lineTo(new Vector2d(-18 * 2.54, 26 * 2.54), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                            .build()
            );
        }
        while (drive.isBusy()) {
            drive.update();
            drive.updatePoseEstimate();

        }
        drive.Colect(0.5);
        asteptare(70);
        drive.Colect(-1);
        systemTime = System.currentTimeMillis();
        drive.setMotorPowers(0.2, 0.2, 0.2, 0.2);
        while (!drive.touchGheara.isPressed() && opModeIsActive() && (systemTime + 500 > System.currentTimeMillis())) {
        }
        drive.servoClamp.setPosition(configs.pozitie_servoClamp_prindere);
        drive.setMotorPowers(0, 0, 0, 0);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .setReversed(true)
                        .splineTo(new Pose2d(-12.5 * 2.54, 38 * 2.54, Math.toRadians(-180)))
                        .splineTo(new Pose2d(12.5 * 2.54, 38 * 2.54, Math.toRadians(-180)))
                        .splineTo(new Pose2d(37 * 2.54, 50 * 2.54, Math.toRadians(-180)))
                        .build()
        );
        drive.dubiousExtensie();
        drive.servoClamp.setPosition(configs.pozitie_servoClamp_desprindere);
        sleep(500);
        drive.dubiousHome();
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .setReversed(false)
                        .splineTo(new Pose2d(15, 38 * 2.54, Math.toRadians(-180)))
                        .splineTo(new Pose2d(0, 38 * 2.54, Math.toRadians(-180)))
                        .build()
        );

        drive.stop = true;
    }


    public Thread inFlightPipelineChange = new Thread(() -> {
        boolean isTerminated = false;
        while (!isTerminated) {
            if (changePipeline) {
                cam.startDetection(new org.firstinspires.ftc.teamcode.opModes.StoneDetector(480, 640));
                isTerminated = true;
            }
        }
    });


    public void colectCub() {
        double power, camPower;
        drive.Colect(-1);
        telemetry.setMsTransmissionInterval(50);
        telemetry.update();
        while (!isCollected) {
            pidCam.setPID(camConfig.kp, camConfig.ki, camConfig.kd);
            drive.update();
            drive.updatePoseEstimate();
            if (drive.getPoseEstimate().getX() > 57 || drive.getPoseEstimate().getY() < 1 || drive.getPoseEstimate().getY() > 60 || drive.getPoseEstimate().getX() < 16) {
                isCrazy = true;
                break;
            }
            telemetry.addData("Cube", cam.skystoneDetectorModified.foundRectangles().get(0));
            telemetry.update();
            power = 0.4;
            isCollected = drive.touchGheara.isPressed();
            pidCam.setPID(camConfig.kp, camConfig.ki, camConfig.kd);
            camPower = pidCam.performPID(cam.skystoneDetectorModified.foundRectangles().get(0).x + (cam.skystoneDetectorModified.foundRectangles().get(0).width) / 2);
            drive.setMotorPowers(power - camPower, power - camPower, power + camPower, power + camPower);
        }
        drive.servoClamp.setPosition(configs.pozitie_servoClamp_prindere
        );
        drive.stopColect();
        drive.setMotorPowers(0, 0, 0, 0);
    }

    public void asteptare(int milisecunde) {
        systemTime = System.currentTimeMillis();
        while (systemTime + milisecunde > System.currentTimeMillis()) ;

    }
}