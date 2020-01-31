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
            scissor.pidPod.setPID(Automatizari_config.kpPod, Automatizari_config.kiPod, Automatizari_config.kdPod);
            scissor.pidPod.setTolerance(Automatizari_config.tolerancePod);
            scissor.pidPod.setSetpoint(Automatizari_config.setpointPod);
            packet.put("PID", scissor.pidPod.performPID(scissor.potentiometruValue));
            packet.put("P_S", Automatizari_config.kpPod * scissor.pidPod.getError());
            packet.put("I_S", Automatizari_config.kiPod * scissor.pidPod.getISum());
            packet.put("D_S", Automatizari_config.kdPod * scissor.pidPod.getDError());
            packet.put("Setpoint", Automatizari_config.setpointPod);
            packet.put("Pot. value", scissor.potentiometruValue);
        }

        //scissor.goScissor(1000);
        //sleep(2000);
        //scissor.goScissor(2000);
        //sleep(2000);
    }
}
