package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ScissorAutoTest extends LinearOpMode {
    Hardware_Scissor_V1 scissor = new Hardware_Scissor_V1();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();
    double podPerfomPid, i = 0;


    @Override
    public void runOpMode() {
        scissor.Init(hardwareMap);
        scissor.servoClamp.setPosition(configs.pozitie_servoClamp_prindere);
        waitForStart();
        scissor.aruncaCuburi();
        scissor.stop = true;
    }

    public void packetPut(int i){
        packet.put("I:", i);
        dashboard.sendTelemetryPacket(packet);
        packet.clearLines();
    }
}
