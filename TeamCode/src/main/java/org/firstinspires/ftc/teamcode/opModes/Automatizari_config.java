package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

@Config
public class Automatizari_config {

    public static double kpAgr = 0.01, kiAgr = 0, kdAgr = 0.01, setpointScissor = 0;//0.003, 0, 0.0025
    public static double kp = 0.003, ki = 0, kd = 0.0025;
    public static double kpPod = 0.008, kiPod = 0, kdPod = 0.01, setpointPod = 1000, maxPodValue = 4000, minPodValue = 0;
    public static double toleranceScissorSt = 150,toleranceScissorDr = 200, tolerancePod = 70,targetVerifications = 5;
}