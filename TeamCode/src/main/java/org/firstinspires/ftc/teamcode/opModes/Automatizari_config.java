package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

@Config
public class Automatizari_config {

    public static double kpAgr = 0.02, kiAgr = 0, kdAgr = 0.05, setpointScissor = 0;//0.003, 0, 0.0025
    public static double kp = 0.003, ki = 0, kd = 0.0025;
    public static double kpPod = 0.004, kiPod = 0, kdPod = 0.008, setpointPod = 1000, maxPodValue = 2000, minPodValue = 0;
    public static double toleranceScissorSt = 150,toleranceScissorDr = 150, tolerancePod = 30,targetVerifications = 50;
}
