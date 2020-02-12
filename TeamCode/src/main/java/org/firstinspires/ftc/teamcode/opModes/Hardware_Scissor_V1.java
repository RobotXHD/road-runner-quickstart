package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

public class Hardware_Scissor_V1 extends LinearOpMode {

    private RevBulkData bulkDataSisteme, bulkDataControl;
    public Servo servoPlatformaSt, servoPlatformaDr, servoCapstone;
    private ExpansionHubEx expansionHubSisteme, expansionHubControl;
    public ExpansionHubMotor scissorDreapta;
    public ExpansionHubMotor scissorStanga;
    public DcMotor motorColectSt, motorColectDr;
    private TouchSensor touchScissorDr, touchScissorSt;
    public boolean stop = false;
    public double verifications, verificationPod, podPerfomPid, powerSc = 0;
    public volatile double encoderDreapta, vitezaDreapta, potentiometruValue;
    public PIDControllerAdevarat pidScissorDr = new PIDControllerAdevarat(0, 0, 0);
    public PIDControllerAdevarat pidScissorDrAgr = new PIDControllerAdevarat(0, 0, 0);
    public PIDControllerAdevarat pidPod = new PIDControllerAdevarat(0, 0, 0);
    public ServoImplEx vexSt, vexDr, servoClamp;
    public AnalogInput potentiometru;
    public double calibScissor = 15.719;
    public int lastPos;


    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private TelemetryPacket packet = new TelemetryPacket();

    public Hardware_Scissor_V1() {
    }

    public void Init(HardwareMap hardwareMap) {
        expansionHubSisteme = hardwareMap.get(ExpansionHubEx.class, configs.expansionHubSistemeName);
        expansionHubControl = hardwareMap.get(ExpansionHubEx.class, configs.expansionHubOdometrieName);

        potentiometru = hardwareMap.analogInput.get(configs.potentiometruName);

        scissorDreapta = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, configs.scissorDrName);
        scissorStanga = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, configs.scissorStName);

        motorColectDr = hardwareMap.get(DcMotor.class, configs.colectDrName);
        motorColectSt = hardwareMap.get(DcMotor.class, configs.colectStName);

        servoPlatformaDr = hardwareMap.servo.get(configs.servoPlatformaDrName);
        servoPlatformaSt = hardwareMap.servo.get(configs.servoPlatformaStName);
        servoClamp = hardwareMap.get(ServoImplEx.class, configs.servoClampName);
        servoCapstone = hardwareMap.get(Servo.class, configs.servoCapstoneName);

        vexDr = hardwareMap.get(ServoImplEx.class, configs.vexDrName);
        vexSt = hardwareMap.get(ServoImplEx.class, configs.vexStName);

        vexDr.setPwmRange(new PwmControl.PwmRange(1000, 2000));
        vexSt.setPwmRange(new PwmControl.PwmRange(1000, 2000));

        scissorDreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        scissorStanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        scissorDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        scissorStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        scissorDreapta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        scissorStanga.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorColectSt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorColectDr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        scissorDreapta.setDirection(DcMotorSimple.Direction.REVERSE);
        scissorStanga.setDirection(DcMotorSimple.Direction.REVERSE);

        touchScissorDr = hardwareMap.touchSensor.get(configs.touchScissorDrName);
        touchScissorSt = hardwareMap.touchSensor.get(configs.touchScissorStName);

        servoCapstone.setPosition(0.7);

        pidScissorDr.setSetpoint(0);
        pidScissorDrAgr.setSetpoint(0);
        pidPod.setSetpoint(Automatizari_config.setpointPod);

        pidScissorDr.setTolerance(Automatizari_config.toleranceScissorDr);
        pidScissorDrAgr.setTolerance(150);
        pidPod.setTolerance(1);

        pidScissorDr.setPID(Automatizari_config.kp, Automatizari_config.ki, Automatizari_config.kd);
        pidScissorDrAgr.setPID(Automatizari_config.kpAgr, Automatizari_config.kiAgr, Automatizari_config.kdAgr);
        pidPod.setPID(Automatizari_config.kpPod, Automatizari_config.kiPod, Automatizari_config.kdPod);

        pidScissorDrAgr.enable();
        pidPod.enable();

        readControl.start();
        readSisteme.start();
        scissorPID.start();
    }

    public void startColect() {
        motorColectDr.setPower(-0.5);
        motorColectSt.setPower(-0.5);
    }

    public void stopColect() {
        motorColectDr.setPower(0);
        motorColectSt.setPower(0);
    }

    public void startColectReverse() {
        motorColectDr.setPower(1);
        motorColectSt.setPower(1);
    }
    public void Colect(double power){
        motorColectDr.setPower(power);
        motorColectSt.setPower(power);
    }

    public void prindrePlate() {
        servoPlatformaDr.setPosition(0);
        servoPlatformaSt.setPosition(1);
    }

    public void desprindrePlate() {
        servoPlatformaDr.setPosition(1);
        servoPlatformaSt.setPosition(0);
    }

    public void goPodRulant(double position) {
        verificationPod = 0;

        if (position < Automatizari_config.minPodValue)
            position = Automatizari_config.minPodValue;
        else if (position > Automatizari_config.maxPodValue)
            position = Automatizari_config.maxPodValue;
        pidPod.setSetpoint(position);

        pidPod.setTolerance(Automatizari_config.tolerancePod);
        do {
            podPerfomPid = pidPod.performPID(potentiometruValue);
            if (podPerfomPid * podPerfomPid > 0.25) podPerfomPid = Math.signum(podPerfomPid) * 0.5;
            vexDr.setPosition(podPerfomPid + 0.5);
            vexSt.setPosition(-podPerfomPid + 0.5);

            verificationPod = pidPod.onTarget() ? verificationPod + 1 : 0;
        } while (verificationPod < Automatizari_config.targetVerifications);
    }

    public void goPodRulant(double position, double triggerTick, double servoPosition) {
        verificationPod = 0;
        boolean isReversed = false;

        if (position < Automatizari_config.minPodValue)
            position = Automatizari_config.minPodValue;
        else if (position > Automatizari_config.maxPodValue)
            position = Automatizari_config.maxPodValue;
        pidPod.setSetpoint(position);

        if (triggerTick < potentiometruValue) {
            isReversed = true;
        }

        pidPod.setTolerance(Automatizari_config.tolerancePod);
        do {
            podPerfomPid = pidPod.performPID(potentiometruValue);
            if (podPerfomPid * podPerfomPid > 0.25) podPerfomPid = Math.signum(podPerfomPid) * 0.5;
            vexDr.setPosition(podPerfomPid + 0.5);
            vexSt.setPosition(-podPerfomPid + 0.5);

            if (isReversed) {
                if (potentiometruValue < triggerTick) {
                    servoClamp.setPosition(servoPosition);
                }
            } else {
                if (potentiometruValue > triggerTick) {
                    servoClamp.setPosition(servoPosition);
                }
            }

            verificationPod = pidPod.onTarget() ? verificationPod + 1 : 0;
        } while (verificationPod < Automatizari_config.targetVerifications);
    }

    public void goScissorAgr(double position) {
        lastPos = (int) encoderDreapta;
        pidScissorDr.disable();
        pidScissorDrAgr.enable();
        pidScissorDrAgr.setSetpoint(position);
        pidScissorDrAgr.performPID(encoderDreapta);
        while (!pidScissorDrAgr.onTarget()) {
        }
    }

    public void goScissor(double position) {
        lastPos = (int) encoderDreapta;
        pidScissorDrAgr.disable();
        pidScissorDr.enable();
        pidScissorDr.setSetpoint(position);
        pidScissorDr.performPID(encoderDreapta);
        if(pidScissorDr.getSetpoint() == 0){
            powerSc = -0.7;
        }
        else{
            powerSc = 0;
        }

        while (!pidScissorDr.onTarget()) {
        }
        if(pidScissorDr.getSetpoint() == 0){
            pidScissorDr.disable();
        }
    }

    public void goCuburi(int cub) {
        //TODO: To stop scissor from crashing hard
        final int ROBOT_SAFE_DISTANCE = 700;
        final int BRIDGE_EXTENDED_POSITION = 2200;
        final int BRIDGE_HOME_POSITION = (int) Automatizari_config.minPodValue;
        final int CUBE_INCREMENT = 180;
        final int[] CUBE_POSITIONS = {0, 220, 420, 550, 720, 0};
        if (cub <= 2) {
            goScissorAgr(ROBOT_SAFE_DISTANCE + (cub - 1) * 100);
            goPodRulant(BRIDGE_EXTENDED_POSITION);
            servoClamp.setPosition(configs.pozitie_servoClamp_desprindere);
            goScissor(CUBE_POSITIONS[cub]);
            goScissorAgr(ROBOT_SAFE_DISTANCE + (cub - 1) * 100);
            goPodRulant(BRIDGE_HOME_POSITION);
            goScissor(0);
        } else {
            if (cub > 4) {
                CUBE_POSITIONS[5] = CUBE_POSITIONS[4] + (cub - 5) * CUBE_INCREMENT;
                goScissorAgr(CUBE_POSITIONS[5] + CUBE_INCREMENT);
                goPodRulant(BRIDGE_EXTENDED_POSITION);
                servoClamp.setPosition(configs.pozitie_servoClamp_desprindere);
                goScissor(CUBE_POSITIONS[5]);
                goScissorAgr(CUBE_POSITIONS[5] + CUBE_INCREMENT + 300);
                goPodRulant(BRIDGE_HOME_POSITION);
                goScissor(0);
            } else {
                goScissorAgr(CUBE_POSITIONS[cub] + CUBE_INCREMENT + 300);
                sleep(1000);
                goPodRulant(BRIDGE_EXTENDED_POSITION);
                servoClamp.setPosition(configs.pozitie_servoClamp_desprindere);
                goScissor(CUBE_POSITIONS[cub]);
                goScissorAgr(CUBE_POSITIONS[cub] + CUBE_INCREMENT + 300);
                goPodRulant(BRIDGE_HOME_POSITION);
                goScissor(0);
            }
        }
    }

    public void aruncaCuburi(){
        goScissorAgr(700);
        goPodRulant(2200, 1500, configs.pozitie_servoClamp_desprindere);
        goPodRulant(Automatizari_config.minPodValue);
        goScissor(0);
    }

    private Thread readControl = new Thread(new Runnable() {
        double pot;

        @Override
        public void run() {
            while (!stop) {
                bulkDataControl = expansionHubControl.getBulkInputData();
                pot = bulkDataControl.getAnalogInputValue(potentiometru);
                potentiometruValue = pot;
            }
        }
    });

    private Thread readSisteme = new Thread(new Runnable() {
        int d, s, scissorStangaOffset;

        @Override
        public void run() {
            while (!stop) {
                if (touchScissorDr.isPressed() || touchScissorSt.isPressed()) {
                    scissorStangaOffset = scissorStanga.getCurrentPosition();
                }
                bulkDataSisteme = expansionHubSisteme.getBulkInputData();
                d = bulkDataSisteme.getMotorCurrentPosition(scissorStanga);
                s = bulkDataSisteme.getMotorVelocity(scissorStanga);
                encoderDreapta = d - scissorStangaOffset;
                vitezaDreapta = s;

            }
        }
    });
    private Thread scissorPID = new Thread(new Runnable() {
        double power = 0, encDr, vDr, brakeDist;

        @Override
        public void run() {
            while (!stop) {
                encDr = encoderDreapta;
                vDr = Math.abs(vitezaDreapta);
                if (pidScissorDr.enabled()) {
                    packet.put("vdr", vDr);
                    packet.clearLines();
                    if (lastPos - pidScissorDr.getSetpoint() > 0) {
                        if (brakeDist + pidScissorDr.getSetpoint() > encDr && pidScissorDr.getSetpoint() < encDr) {
                            pidScissorDr.performPID(encDr);
                            power = 0;
                            packet.put("brakeDist", brakeDist);
                        } else {
                            brakeDist = 0.08 * vDr;
                            power = pidScissorDr.performPID(encDr);
                        }
                    } else {
                        power = pidScissorDr.performPID(encDr);
                    }
                    dashboard.sendTelemetryPacket(packet);
                    packet.clearLines();
                } else if (pidScissorDrAgr.enabled()) {
                    power = pidScissorDrAgr.performPID(encDr);
                } else if(!touchScissorSt.isPressed() || !touchScissorDr.isPressed()){
                    power = powerSc;
                }
                else{
                    powerSc = 0;
                }
                scissorStanga.setPower(power);
                scissorDreapta.setPower(power);
            }
        }
    });

    public void runOpMode() {
    }
}
