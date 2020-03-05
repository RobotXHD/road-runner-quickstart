package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore .hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
@TeleOp
public class CorectieCosBrat extends OpMode {
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private AnalogInput potentiometru;
    private ServoImplEx servoBrat;
    private double pozPot,pozServo;
    public double initPos = 0;
    private boolean stop = false;
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        potentiometru = hardwareMap.analogInput.get(configs.potentiometruName);
        servoBrat = hardwareMap.get(ServoImplEx.class, "brat");
        initPos = potentiometru.getVoltage() * 1000;
        telemetry.setMsTransmissionInterval(25);
        auto.start();
    }

    public Thread auto = new Thread(()->{
        while(!stop){
            pozPot = ((potentiometru.getVoltage() * 1000 - initPos) * 0.03601609657947686116700201207243) < 0 ? 0 : (potentiometru.getVoltage() * 1000 - initPos) * 0.03601609657947686116700201207243;
            if(pozPot < 0){
                pozPot = 0;
            }
            pozServo = 0.99 - Automatizari_config.k * Math.
                    acos(1-((pozPot)/configs.lungime_Brat_verticala));
            servoBrat.setPosition(pozServo);
        }
    });

    @Override
    public void loop() {
        telemetry.addData("Init", initPos);
        telemetry.addData("poz", potentiometru.getVoltage() * 1000);
        telemetry.addData("pozPot", pozPot);
        telemetry.addData("pozServo",pozServo);
        telemetry.update();
    }
    public void stop(){
        stop = true;
    }
}
