package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class ScissorAutoTest extends LinearOpMode {
    Hardware_Scissor_V1 scissor = new Hardware_Scissor_V1();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();
    @Override
    public void runOpMode() {
        scissor.Init(hardwareMap);
        waitForStart();

        while (opModeIsActive()){
            packet.put("Pot", scissor.potentiometru.getVoltage());
            dashboard.sendTelemetryPacket(packet);
            packet.clearLines();
            scissor.motorColectDr.setPower(1);
            scissor.motorColectSt.setPower(1);
        }

        //scissor.goScissor(1000);
        //sleep(2000);
        //scissor.goScissor(2000);
        //sleep(2000);
    }
}
