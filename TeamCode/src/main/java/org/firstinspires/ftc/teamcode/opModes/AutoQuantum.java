package org.firstinspires.ftc.teamcode.opModes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class AutoQuantum extends LinearOpMode {
    Hardware_Skybot_V3 r = new Hardware_Skybot_V3(true);

    @Override
    public void runOpMode(){
        r.Init(hardwareMap);
        waitForStart();
        r.gotoY(5000,1);
        sleep(1000);
    }
}