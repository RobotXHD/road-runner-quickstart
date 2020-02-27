package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore .hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ServoImplEx;
@TeleOp
public class CorectieCosBrat extends OpMode {
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private AnalogInput potentiometru;
    private ServoImplEx servoBrat;
    private double pozPot,pozServo;
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        potentiometru = hardwareMap.analogInput.get(configs.potentiometruName);
        servoBrat = hardwareMap.get(ServoImplEx.class, "brat");
    }

    @Override
    public void loop() {
        pozPot = potentiometru.getVoltage() * 1000;
        pozServo = Math.acos(pozPot);
        servoBrat.setPosition(pozServo);
        telemetry.addData("pozPot",pozPot);
        telemetry.addData("pozServo",pozServo);
        telemetry.update();
    }
}
