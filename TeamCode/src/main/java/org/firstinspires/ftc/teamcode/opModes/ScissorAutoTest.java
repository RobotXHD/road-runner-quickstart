package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class ScissorAutoTest extends LinearOpMode {
    Hardware_Scissor_V1 scissor = new Hardware_Scissor_V1();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    double podPerfomPid;
    TelemetryPacket packet = new TelemetryPacket();
    @Override
    public void runOpMode() {
        scissor.Init(hardwareMap);
        waitForStart();

        while (opModeIsActive()){
            scissor.pidPod.setPID(Automatizari_config.kpPod, Automatizari_config.kiPod, Automatizari_config.kdPod);
            scissor.pidPod.setTolerance(Automatizari_config.tolerancePod);
            if(Automatizari_config.setpointPod<Automatizari_config.minPodValue)Automatizari_config.setpointPod =Automatizari_config.minPodValue;
            else if(Automatizari_config.setpointPod>Automatizari_config.maxPodValue)Automatizari_config.setpointPod = Automatizari_config.maxPodValue;
            scissor.pidPod.setSetpoint(Automatizari_config.setpointPod);
            podPerfomPid = scissor.pidPod.performPID(scissor.potentiometruValue);
            if(podPerfomPid*podPerfomPid<0.25)podPerfomPid = Math.signum(podPerfomPid)*0.5;

            scissor.vexDr.setPosition(podPerfomPid + 0.5);
            scissor.vexSt.setPosition(-podPerfomPid+ 0.5);
            packet.put("PID_Dr", scissor.pidPod.performPID(scissor.potentiometruValue) + 0.5);
            packet.put("PID_St", -scissor.pidPod.performPID(scissor.potentiometruValue) + 0.5);
            packet.put("P_S", Automatizari_config.kpPod * scissor.pidPod.getError());
            packet.put("I_S", Automatizari_config.kiPod * scissor.pidPod.getISum());
            packet.put("D_S", Automatizari_config.kdPod * scissor.pidPod.getDError());
            packet.put("Setpoint", Automatizari_config.setpointPod);
            packet.put("Pot. value", scissor.potentiometruValue);
            dashboard.sendTelemetryPacket(packet);
            packet.clearLines();
        }

        //scissor.goScissor(1000);
        //sleep(2000);
        //scissor.goScissor(2000);
        //sleep(2000);
    }
}
