package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
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

    private RevBulkData bulkDataControl;
    public Servo servoPlatformaSt, servoPlatformaDr, servoCapstone;
    public ExpansionHubEx expansionHubControl;
    public ExpansionHubMotor scissorDreapta;
    public ExpansionHubMotor scissorStanga;
    public DcMotor motorColectSt, motorColectDr;
    public TouchSensor touchScissorDr, touchScissorSt, touchGheara;
    public boolean stop = false;
    public double verifications, verificationPod, podPerfomPid, powerSc = 0;
    public volatile double encoderDreapta, vitezaDreapta, potentiometruValue, lastPotVal;
    public PIDControllerAdevarat pidScissorDr = new PIDControllerAdevarat(0, 0, 0);
    public PIDControllerAdevarat pidScissorDrAgr = new PIDControllerAdevarat(0, 0, 0);
    public PIDControllerAdevarat pidPod = new PIDControllerAdevarat(0, 0, 0);
    public PIDControllerAdevarat pidCam = new PIDControllerAdevarat(camConfig.kp,camConfig.ki,camConfig.kd);
    public ServoImplEx vexSt, vexDr, servoClamp;
    public AnalogInput potentiometru;
    public double calibScissor = 15.719;
    public double lastPos, potSpeed, time, lastTime;
    public volatile boolean NOTDUCK = false;


    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private TelemetryPacket packet = new TelemetryPacket();

    public Hardware_Scissor_V1() {
    }

    public void Init(HardwareMap hardwareMap) {
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

        scissorDreapta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        scissorStanga.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorColectSt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorColectDr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        scissorDreapta.setDirection(DcMotorSimple.Direction.REVERSE);
        scissorStanga.setDirection(DcMotorSimple.Direction.REVERSE);

        touchScissorDr = hardwareMap.touchSensor.get(configs.touchScissorDrName);
        touchScissorSt = hardwareMap.touchSensor.get(configs.touchScissorStName);
        touchGheara = hardwareMap.touchSensor.get(configs.touchGhearaName);

        servoCapstone.setPosition(0.7);

        pidCam.setSetpoint(100);

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

        lastTime = System.nanoTime();
        time = System.nanoTime();

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
        servoPlatformaDr.setPosition(0.8);
        servoPlatformaSt.setPosition(0.2);
    }

    public void goPodRulant(double position) {
        verificationPod = 0;
        boolean isMoving = false;

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

            if(Math.abs(potSpeed) > 0.5){
                isMoving = true;
            }
            if(isMoving && Math.abs(potSpeed) < 0.2 || pidPod.onTarget()){
                verificationPod++;
            }
            else{
                verificationPod = 0;
            }
        } while (verificationPod < Automatizari_config.targetVerifications);
        vexSt.setPosition(0.5);
        vexDr.setPosition(0.5);
    }

    public void goPodRulant(double position, double triggerTick, double servoPosition) {
        verificationPod = 0;
        boolean isReversed = false;
        boolean isMoving = false;

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

            if(Math.abs(potSpeed) > 0.5){
                isMoving = true;
            }
            if(isMoving && Math.abs(potSpeed) < 0.2 || pidPod.onTarget()){
                verificationPod++;
            }
            else{
                verificationPod = 0;
            }

        } while (verificationPod < Automatizari_config.targetVerifications);
        vexSt.setPosition(0.5);
        vexDr.setPosition(0.5);
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
        sleep(300);
        if(pidScissorDr.getSetpoint() == 0){
            pidScissorDr.disable();
        }
        while(!touchScissorDr.isPressed() && !touchScissorSt.isPressed()){

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
        NOTDUCK = true;
        goScissorAgr(700);
        goPodRulant(3000, 1700, configs.pozitie_servoClamp_desprindere);
        goPodRulant(Automatizari_config.minPodValue);
        goScissor(0);
        NOTDUCK = false;
    }

    private Thread readControl = new Thread(new Runnable() {
        double pot;

        @Override
        public void run() {
            while (!stop) {
                if(NOTDUCK) {
                    bulkDataControl = expansionHubControl.getBulkInputData();
                    pot = bulkDataControl.getAnalogInputValue(potentiometru);
                    lastTime = time;
                    lastPotVal = potentiometruValue;
                    potentiometruValue = pot;
                    time = System.nanoTime();
                    if (Math.abs(pot - lastPotVal) < 3) {
                        lastPotVal = pot;
                    }
                    potSpeed = (pot - lastPotVal) / (time - lastTime) * 1000000;
                    sleep(50);
                }
            }
        }
    });

    private Thread readSisteme = new Thread(new Runnable() {
        double d, s, scissorStangaOffset;

        @Override
        public void run() {
            while (!stop) {
                if(NOTDUCK) {
                    d = scissorStanga.getCurrentPosition();
                    if (touchScissorDr.isPressed() || touchScissorSt.isPressed()) {
                        scissorStangaOffset = d;
                    }
                    s = scissorStanga.getVelocity();
                    encoderDreapta = d - scissorStangaOffset;
                    vitezaDreapta = s;
                }
            }
        }
    });
    private Thread scissorPID = new Thread(new Runnable() {
        double power = 0, encDr, vDr, brakeDist;

        @Override
        public void run() {
            while (!stop) {
                if (NOTDUCK) {
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
                    } else if (!touchScissorSt.isPressed() || !touchScissorDr.isPressed()) {
                        power = powerSc;
                    } else {
                        powerSc = 0;
                        power = powerSc;
                    }
                    scissorStanga.setPower(power);
                    scissorDreapta.setPower(power);
                }
            }
        }
    });

    public void runOpMode() {
    }
}
