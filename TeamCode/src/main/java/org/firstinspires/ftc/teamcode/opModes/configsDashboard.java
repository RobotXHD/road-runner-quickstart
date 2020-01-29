package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.config.Config;

@Config
public class configsDashboard {
    public static double kpr= 0.1, kir = 0, kdr = 0, targetrotatie = 0;//0.3 0 6500
    public static double ku = 0.17, tu = 0.256637168141593, ti = 0, td = tu/8;
    public static double znkp = 0.8*ku, znki = 0, znkd = 3 * ku * tu / 40;
    public static boolean pid1 = true, pidzn = false;
    //public static double kpt= 0, kit = 0, kdt = 6500, targetTranslatie = 0;
}
