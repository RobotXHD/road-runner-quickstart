package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class ScissorAutoTest extends LinearOpMode {
    Hardware_Scissor_V1 scissor = new Hardware_Scissor_V1();
    @Override
    public void runOpMode() {
        scissor.Init(hardwareMap);
        waitForStart();
        scissor.goScissor(1000);
        sleep(2000);
        scissor.goScissor(2000);
        sleep(2000);
    }
}
