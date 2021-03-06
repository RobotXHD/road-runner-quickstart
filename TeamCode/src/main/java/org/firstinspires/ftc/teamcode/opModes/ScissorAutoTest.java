package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

public class ScissorAutoTest extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();
    @Override
    public void runOpMode() {
        SampleMecanumDriveREVOptimized scissor = new SampleMecanumDriveREVOptimized(hardwareMap);
        scissor.Init(hardwareMap);

        waitForStart();

        scissor.NOTDUCK = true;
        scissor.goScissorAgr(600);
        scissor.goPodRulant(2500);
        sleep(2000);
        scissor.NOTDUCK = false;

       /* while (opModeIsActive()){
            scissor.pidPod.setPID(Automatizari_config.kpPod, Automatizari_config.kiPod, Automatizari_config.kdPod);
            scissor.pidPod.setTolerance(Automatizari_config.tolerancePod);

            if(Automatizari_config.setpointPod<Automatizari_config.minPodValue)
                Automatizari_config.setpointPod =Automatizari_config.minPodValue;
            else if(Automatizari_config.setpointPod>Automatizari_config.maxPodValue)
                Automatizari_config.setpointPod = Automatizari_config.maxPodValue;

            scissor.pidPod.setSetpoint(Automatizari_config.setpointPod);

            podPerfomPid = scissor.pidPod.performPID(scissor.potentiometruValue);
            if(podPerfomPid*podPerfomPid>0.25)podPerfomPid = Math.signum(podPerfomPid)*0.5;

            scissor.vexDr.setPosition(podPerfomPid + 0.5);
            scissor.vexSt.setPosition(-podPerfomPid+ 0.5);

            packet.put("PID_Dr", podPerfomPid + 0.5);
            packet.put("PID_St", -podPerfomPid + 0.5);
            packet.put("P_S", Automatizari_config.kpPod * scissor.pidPod.getError());
            packet.put("I_S", Automatizari_config.kiPod * scissor.pidPod.getISum());
            packet.put("D_S", Automatizari_config.kdPod * scissor.pidPod.getDError());
            packet.put("Setpoint", Automatizari_config.setpointPod);
            packet.put("Pot. value", scissor.potentiometruValue);
            dashboard.sendTelemetryPacket(packet);
            packet.clearLines();
        }*/

        //scissor.goScissor(1000);
        //sleep(2000);
        //scissor.goScissor(2000);
        //sleep(2000);
    }
}
