package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Automatizari_config {
    public static double kp = 0.003, ki = 0, kd = 0.0025, setpointScissor = 0, Setpoint = 0;
    public static double kpPod = 0, kiPod = 0, kdPod = 0, setpointPod = 3;
    public static double toleranceScissorSt = 20,toleranceScissorDr = 20, tolerancePod = 0.1,targetVerifications = 50;
}
