package org.firstinspires.ftc.teamcode.opModes;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous
public class AutoTestV1 extends LinearOpMode {
    Hardware_Cam_Red cam = new Hardware_Cam_Red();
    int caz ;
    @Override
    public void runOpMode() {
        cam.Init(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Ceva: ", cam.stoneDetectorModified.foundScreenPositions().get(0).y);

            if(cam.stoneDetectorModified.foundScreenPositions().get(0).y > 90){
                telemetry.addData("Position", "LEFT");
                caz = 0;
            }
            else if(cam.stoneDetectorModified.foundScreenPositions().get(0).y> 80    )      {
                telemetry.addData("Position", "CENTRE");
                caz = 1;
            }
            else {
                telemetry.addData("Position", "RIGHT");
                caz = -1;

            }
            telemetry.update();
        }
    }
}