package org.firstinspires.ftc.teamcode.opModes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

public class Hardware_Scissor_V1 extends LinearOpMode {

    private RevBulkData bulkData;
    private ExpansionHubEx expansionHubSisteme;
    private ExpansionHubMotor scissorDreapta;
    private ExpansionHubMotor scissorStanga;
    private TouchSensor touchScissorDr, touchScissorSt;
    public boolean stop = false;
    public double verifications;
    private volatile double encoderDreapta, encoderStranga;
    private PIDControllerAdevarat pidScissorDr = new PIDControllerAdevarat(0, 0, 0);
    private PIDControllerAdevarat pidScissorSt = new PIDControllerAdevarat(0, 0, 0);

    public Hardware_Scissor_V1(){}

    public void Init(HardwareMap hard) {
        expansionHubSisteme = hard.get(ExpansionHubEx.class, configs.expansionHubSistemeName);
        scissorDreapta = (ExpansionHubMotor) hard.get(DcMotorEx.class, configs.scissorDrName);
        scissorStanga = (ExpansionHubMotor) hard.get(DcMotorEx.class, configs.scissorStName);

        scissorDreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        scissorStanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        scissorDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        scissorStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        scissorDreapta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        scissorStanga.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        scissorDreapta.setDirection(DcMotorSimple.Direction.REVERSE);
        scissorStanga.setDirection(DcMotorSimple.Direction.REVERSE);

        touchScissorDr = hard.touchSensor.get(configs.touchScissorDrName);
        touchScissorSt = hard.touchSensor.get(configs.touchScissorStName);

        pidScissorDr.setSetpoint(0);
        pidScissorSt.setSetpoint(0);

        pidScissorDr.setTolerance(Automatizari_config.toleranceScissorDr);
        pidScissorSt.setTolerance(Automatizari_config.toleranceScissorSt);

        pidScissorDr.setPID(Automatizari_config.kp, Automatizari_config.ki, Automatizari_config.kd);
        pidScissorSt.setPID(Automatizari_config.kp, Automatizari_config.ki, Automatizari_config.kd);


        pidScissorDr.enable();
        pidScissorSt.enable();
        read.start();
    }

    public void goScissor(double position){
        verifications = 0;

        pidScissorDr.setSetpoint(position);
        pidScissorSt.setSetpoint(position);

        pidScissorSt.setTolerance(Automatizari_config.toleranceScissorSt);
        pidScissorDr.setTolerance(Automatizari_config.toleranceScissorDr);
        do{
            scissorStanga.setPower(pidScissorSt.performPID(encoderStranga));
            scissorDreapta.setPower(pidScissorDr.performPID(encoderDreapta));
            verifications = pidScissorSt.onTarget() ? verifications + 1 : 0;
        }while(verifications < Automatizari_config.targetVerifications);
    }


    private Thread Scissor = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {

                pidScissorDr.setSetpoint(Automatizari_config.setpointScissor);
                pidScissorSt.setSetpoint(Automatizari_config.setpointScissor);

                scissorDreapta.setPower(pidScissorDr.performPID(encoderDreapta));
                scissorStanga.setPower(pidScissorSt.performPID(encoderStranga));

                if (touchScissorDr.isPressed()) {
                    scissorDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    scissorDreapta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    pidScissorDr.setSetpoint(0);
                }
                if (touchScissorSt.isPressed()) {
                    scissorStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    scissorStanga.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    pidScissorSt.setSetpoint(0);
                }
            }
        }
    });
    private Thread read = new Thread(new Runnable() {
        double s,d;
        @Override
        public void run() {
            while(!stop){
                bulkData = expansionHubSisteme.getBulkInputData();
                s = bulkData.getMotorCurrentPosition(scissorStanga);
                d = bulkData.getMotorCurrentPosition(scissorDreapta);

                encoderDreapta = d;
                encoderStranga = s;
            }
        }
    });


    public void runOpMode() {
    }
}
