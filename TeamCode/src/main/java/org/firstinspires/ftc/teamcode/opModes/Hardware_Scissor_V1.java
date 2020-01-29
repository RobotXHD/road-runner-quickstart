package org.firstinspires.ftc.teamcode.opModes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

public class Hardware_Scissor_V1 extends LinearOpMode {

    private RevBulkData bulkData;
    private ExpansionHubEx expansionHubSisteme;
    private ExpansionHubMotor scissorDreapta;
    private ExpansionHubMotor scissorStanga;
    public DcMotor motorColectSt, motorColectDr;
    private TouchSensor touchScissorDr, touchScissorSt;
    public boolean stop = false;
    public double verifications;
    private volatile double encoderDreapta;
    private PIDControllerAdevarat pidScissorDr = new PIDControllerAdevarat(0, 0, 0);
    public ServoImplEx vexSt, vexDr;
    public AnalogInput potentiometru;

    public Hardware_Scissor_V1(){}

    public void Init(HardwareMap hardwareMap) {
        expansionHubSisteme = hardwareMap.get(ExpansionHubEx.class, configs.expansionHubSistemeName);

        scissorDreapta = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, configs.scissorDrName);
        scissorStanga = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, configs.scissorStName);

        motorColectDr = hardwareMap.get(DcMotor.class, configs.colectDrName);
        motorColectSt = hardwareMap.get(DcMotor.class, configs.colectStName);

        vexDr = hardwareMap.get(ServoImplEx.class, "vexDr");
        vexSt = hardwareMap.get(ServoImplEx.class, "vexSt");
        vexDr.setPwmRange(new PwmControl.PwmRange(1000, 2000));
        vexSt.setPwmRange(new PwmControl.PwmRange(1000, 2000));
        potentiometru = hardwareMap.analogInput.get("pot");

        scissorDreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        scissorStanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        scissorDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        scissorStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        scissorDreapta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        scissorStanga.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        scissorDreapta.setDirection(DcMotorSimple.Direction.REVERSE);
        scissorStanga.setDirection(DcMotorSimple.Direction.REVERSE);

        touchScissorDr = hardwareMap.touchSensor.get(configs.touchScissorDrName);
        touchScissorSt = hardwareMap.touchSensor.get(configs.touchScissorStName);

        pidScissorDr.setSetpoint(0);

        pidScissorDr.setTolerance(Automatizari_config.toleranceScissorDr);

        pidScissorDr.setPID(Automatizari_config.kp, Automatizari_config.ki, Automatizari_config.kd);

        pidScissorDr.enable();
        read.start();
    }

    public void goScissor(double position){
        verifications = 0;

        pidScissorDr.setSetpoint(position);

        pidScissorDr.setTolerance(Automatizari_config.toleranceScissorDr);
        do{
            scissorStanga.setPower(pidScissorDr.performPID(encoderDreapta));
            scissorDreapta.setPower(pidScissorDr.performPID(encoderDreapta));
            verifications = pidScissorDr.onTarget() ? verifications + 1 : 0;
        }while(verifications < Automatizari_config.targetVerifications);
    }


    private Thread Scissor = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {

                pidScissorDr.setSetpoint(Automatizari_config.setpointScissor);

                scissorDreapta.setPower(pidScissorDr.performPID(encoderDreapta));

                if (touchScissorDr.isPressed()) {
                    scissorDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    scissorDreapta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    pidScissorDr.setSetpoint(0);
                }
                if (touchScissorSt.isPressed()) {
                    scissorStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    scissorStanga.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
                d = bulkData.getMotorCurrentPosition(scissorDreapta);

                encoderDreapta = d;
            }
        }
    });


    public void runOpMode() {
    }
}
