package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;


public class Automatizari extends OpMode {
    private FtcDashboard ftcDashboard = FtcDashboard.getInstance();
    private RevBulkData bulkData;
    private ExpansionHubEx expansionHubSisteme;
    private ExpansionHubMotor scissorDreapta;
    private ExpansionHubMotor scissorStanga;
    private TouchSensor touchScissorDr, touchScissorSt;
    private AnalogInput potentiometru;
    public Servo vexDr, vexSt,servoClamp;
    public boolean stop = false;
    private double offsetDreapta, offsetStanga;
    private volatile double encoderDreapta, encoderStranga, encpot;
    private PIDControllerAdevarat pidScissorDr = new PIDControllerAdevarat(0, 0, 0);
    private PIDControllerAdevarat pidScissorSt = new PIDControllerAdevarat(0, 0, 0);

    @Override

    public void init() {
        expansionHubSisteme = hardwareMap.get(ExpansionHubEx.class, configs.expansionHubSistemeName);
        scissorDreapta = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, configs.scissorDrName);
        scissorStanga = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, configs.scissorStName);

        scissorDreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        scissorStanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        scissorDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        scissorStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        scissorDreapta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        scissorStanga.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        scissorDreapta.setDirection(DcMotorSimple.Direction.REVERSE);
        scissorStanga.setDirection(DcMotorSimple.Direction.REVERSE);

         servoClamp = hardwareMap.servo.get(configs.servoClampName);
         vexDr = hardwareMap.servo.get(configs.vexDrName);
         vexSt = hardwareMap.servo.get(configs.vexStName);

        touchScissorDr = hardwareMap.touchSensor.get(configs.touchScissorDrName);
        touchScissorSt = hardwareMap.touchSensor.get(configs.touchScissorStName);

         potentiometru = hardwareMap.analogInput.get(configs.potentiometruName);

         pidScissorDr.setSetpoint(0);
         pidScissorSt.setSetpoint(0);
         pidScissorDr.enable();
         pidScissorSt.enable();
         read.start();
         Scissor.start();
    }


    private Thread Scissor = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {

                pidScissorDr.setPID(Automatizari_config.kp, Automatizari_config.ki, Automatizari_config.kd);
                pidScissorSt.setPID(Automatizari_config.kp, Automatizari_config.ki, Automatizari_config.kd);
                pidScissorDr.setSetpoint(Automatizari_config.setpointScissor);
                pidScissorSt.setSetpoint(Automatizari_config.setpointScissor);
                pidScissorDr.setTolerance(Automatizari_config.toleranceScissorDr);
                pidScissorSt.setTolerance(Automatizari_config.toleranceScissorSt);

                scissorDreapta.setPower(pidScissorDr.performPID(encoderDreapta));
                scissorStanga.setPower(pidScissorSt.performPID(encoderStranga));

                if (touchScissorDr.isPressed()) {
                    scissorDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    pidScissorDr.setSetpoint(0);
                }
                if (touchScissorSt.isPressed()) {
                    scissorStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    pidScissorSt.setSetpoint(0);
                }

               /* TelemetryPacket packet = new TelemetryPacket();
                packet.put("EncSt", encoderStranga);
                packet.put("EncDr", encoderDreapta);
                packet.put("P_S", Automatizari_config.kp * pidScissorSt.getError());
                packet.put("P_D", Automatizari_config.kp * pidScissorDr.getError());
                packet.put("I_S", Automatizari_config.ki * pidScissorSt.getISum());
                packet.put("I_D", Automatizari_config.ki * pidScissorDr.getISum());
                packet.put("D_S", Automatizari_config.kd * pidScissorSt.getDError());
                packet.put("D_D", Automatizari_config.kd * pidScissorDr.getDError());
                packet.put("onTargetDr", pidScissorDr.onTarget());
                packet.put("onTargetSt", pidScissorSt.onTarget());
                packet.put("Sp", Automatizari_config.setpoint);
                packet.put("offsetDr", touchScissorDr.isPressed());
                packet.put("offsetSt", touchScissorSt.isPressed());
                ftcDashboard.sendTelemetryPacket(packet);*/
            }
        }
    });
    private Thread read = new Thread(new Runnable() {
        double s,d, encPotntiometru;
        @Override
        public void run() {
            while(!stop){
                bulkData = expansionHubSisteme.getBulkInputData();
                s = bulkData.getMotorCurrentPosition(scissorStanga);
                d = bulkData.getMotorCurrentPosition(scissorDreapta);

                encoderDreapta = d;
                encoderStranga = s;

                encPotntiometru = bulkData.getAnalogInputValue(potentiometru);
                encpot= encPotntiometru;
                TelemetryPacket packet = new TelemetryPacket();
                packet.put("Pot", encpot);
                ftcDashboard.sendTelemetryPacket(packet);
            }
        }
    });

    private Thread podRulant = new Thread(new Runnable() {
        @Override
        public void run(){
            while(!stop){

            }
        }
    });
    @Override
    public void loop() {
    }

    @Override
    public void stop() {
        stop = true;
        super.stop();
    }
}
