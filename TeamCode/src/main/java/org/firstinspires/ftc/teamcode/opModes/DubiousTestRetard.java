package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

@TeleOp
public class DubiousTestRetard extends LinearOpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();
    public boolean pid = false;
    public double encDr, encSt, encSp;
    public RevBulkData bulkData;
    public ExpansionHubMotor encoderDreapta, encoderSpate, encoderStanga;
    public ExpansionHubEx expansionHub;
    public ExpansionHubMotor motorss, motorsf, motords, motordf;
    public DcMotor motorColectSt, motorColectDr;
    public double encoderDrTotal = 0, encoderStTotal = 0, encoderSpTotal = 0;
    public double rotatie = 0, rotatieTotala = 0, deltaRotatie = 0, deltaTranslatie = 0;
    public double wheelDiameter = 60;
    public double powerDr, powerSt,powerRotatie,powerTranslatie;
    public long calculatedTicks = 0;
    public Servo servoPlatformaSt, servoPlatformaDr;
    private double timeChangeRotatie, lastTimeRotatie, lastDeltaRotatie, deltaSumRotatie, dDeltaRotatie,startTimeRotatie;
    private double timeChangeTranslatie, lastTimeTranslatie, lastDeltaTranslatie,deltaSumTranslatie, dDeltaTranslatie,startTimeTranslatie;
    private double timeChange, lastTime, lastDelta, deltaSum, dDelta;
    public double ticksPerDegree = 70.2335277777777;
    public long PIDrotatieLoopTime = 10000;//ns

    public double startTime, c,fps;

    @Override
    public void runOpMode() {
        expansionHub = hardwareMap.get(ExpansionHubEx.class, configs.expansionHubOdometrieName);
        motorss = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, configs.ssName);
        motords = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, configs.dsName);
        motorsf = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, configs.sfName);
        motordf = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, configs.dfName);

        encoderDreapta = motorss;
        encoderSpate = motorsf;
        encoderStanga = motordf;

        motordf.setPower(0);
        motords.setPower(0);
        motorsf.setPower(0);
        motorss.setPower(0);

        motordf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motords.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorsf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorss.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motordf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motords.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorsf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorss.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motordf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motords.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorsf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorss.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motords.setDirection(DcMotorSimple.Direction.REVERSE);
        motordf.setDirection(DcMotorSimple.Direction.REVERSE);

        encoderRead.start();
        while(!isStarted()){
            telemetry.addData("encDr", encDr);
            telemetry.addData("encSt", encSt);
            telemetry.addData("encSp", encSp);
            telemetry.update();
        }
        waitForStart();
        PIDrotatie.start();
        PIDrotatie2.start();
        while(!isStopRequested()){}
    }
    private Thread encoderRead = new Thread(new Runnable() {
        long st, dr, sp;
        @Override
        public void run() {
            while(!isStopRequested()){
                bulkData = expansionHub.getBulkInputData();
                st = bulkData.getMotorCurrentPosition(encoderStanga);
                sp = bulkData.getMotorCurrentPosition(encoderSpate);
                dr = bulkData.getMotorCurrentPosition(encoderDreapta);
                encDr = dr;
                encSp = sp;
                encSt = st;
                rotatie = ((dr - st)/2.0)/ticksPerDegree;
            }
        }
    });

    private Thread PIDrotatie = new Thread(new Runnable() {
        @Override
        public void run() {
            while(!isStopRequested()){
                if(configsDashboard.pid1){
                    deltaRotatie = configsDashboard.targetrotatie - rotatie;
                    startTimeRotatie = System.nanoTime();
                    timeChangeRotatie = (startTimeRotatie - lastTimeRotatie)/1000.0;
                    deltaSumRotatie += (deltaRotatie * timeChangeRotatie);
                    if(deltaRotatie * deltaRotatie < 0.01){
                        deltaSumRotatie = 0;
                    }
                    dDeltaRotatie = (deltaRotatie - lastDeltaRotatie)/timeChangeRotatie;
                    lastDeltaRotatie = deltaRotatie;
                    lastTimeRotatie = startTimeRotatie;
                    powerRotatie = configsDashboard.kpr * deltaRotatie + configsDashboard.kir * deltaSumRotatie + configsDashboard.kdr * dDeltaRotatie;
                    power(powerRotatie, powerRotatie, -powerRotatie, -powerRotatie);
                    TelemetryPacket packet = new TelemetryPacket();
                    packet.put("Delta Rotatie", deltaRotatie);
                    packet.put("D Rotatie", configsDashboard.kdr * dDeltaRotatie);
                    packet.put("I Rotatie", configsDashboard.kir * deltaSumRotatie);
                    packet.put("P Rotatie", configsDashboard.kpr * deltaRotatie);
                    packet.put("PID Rotatie", powerRotatie);
                    dashboard.sendTelemetryPacket(packet);
/*
                    deltaTranslatie = configsDashboard.targetTranslatie - ((encSp+encDr)/2);
                    startTimeTranslatie = System.nanoTime();
                    timeChangeTranslatie = (startTimeTranslatie - lastDeltaTranslatie)/1000.0;
                    deltaSumTranslatie += (deltaTranslatie * timeChangeTranslatie);
                    dDeltaTranslatie = (deltaTranslatie - lastDeltaTranslatie)/timeChangeTranslatie;
                    lastDeltaTranslatie = deltaTranslatie;
                    lastTimeTranslatie = startTimeTranslatie;
                    powerTranslatie = configsDashboard.kpt*deltaTranslatie + configsDashboard.kit*deltaSumTranslatie + configsDashboard.kdt* dDeltaTranslatie;
                    packet.put("DeltaTranslatie",deltaTranslatie);
                    packet.put("P Translatie",configsDashboard.kpt*deltaTranslatie);
                    packet.put("I Translatie",configsDashboard.kit*deltaSumTranslatie);
                    packet.put("D Translatie",configsDashboard.kdt*dDeltaTranslatie);
                    packet.put("PID Translatie",powerTranslatie);

                    sleep(5);

 */
                }
            }
        }
    });

    private Thread PIDrotatie2 = new Thread(new Runnable() {
        @Override
        public void run() {
            while(!isStopRequested()){
                if(configsDashboard.pidzn){
                    deltaRotatie = configsDashboard.targetrotatie - rotatie;
                    startTimeRotatie = System.nanoTime();
                    timeChangeRotatie = (startTimeRotatie - lastTimeRotatie)/1000.0;
                    deltaSumRotatie += (deltaRotatie * timeChangeRotatie);
                    dDeltaRotatie = (deltaRotatie - lastDeltaRotatie)/timeChangeRotatie;
                    lastDeltaRotatie = deltaRotatie;
                    lastTimeRotatie = startTimeRotatie;
                    powerRotatie = configsDashboard.znkp * (deltaRotatie + configsDashboard.td * dDeltaRotatie);
                    power(powerRotatie, powerRotatie, -powerRotatie, -powerRotatie);
                    TelemetryPacket packet = new TelemetryPacket();
                    packet.put("Delta Rotatie", deltaRotatie);
                    packet.put("PID Rotatie", powerRotatie);
                    dashboard.sendTelemetryPacket(packet);
                }
            }
        }
    });
/*
    public void rotatie(int grade, double putere) {
        double calculatedTicks = grade * 70.2335277777777;
        boolean reverse = true;
        encoderDrTotal -= calculatedTicks;
        encoderStTotal += calculatedTicks;
        if (grade < 0) {
            reverse = false;
        }
        delta = rotatie - grade;
        telemetry.addData("DELTA: ",delta);
        telemetry.update();
        sleep(1000);
        while (delta > 0.01 || delta < -0.01) {
            delta = rotatie - grade;
            telemetry.addData("delta", delta);
            telemetry.addData("PID", PID(delta, kpr, kir, kdr));
            telemetry.addData("rotatie",rotatie);
            telemetry.update();
        }
    }

 */
    private void power(double ds, double df, double ss, double sf) {
        motordf.setPower(df);
        motorss.setPower(ss);
        motorsf.setPower(sf);
        motords.setPower(ds);
    }
    public double PID(double delta, double kp, double ki, double kd) {
        startTime = System.nanoTime();
        timeChange = (startTime - lastTime)/1000.0;
        deltaSum += (delta * timeChange);
        dDelta = (delta - lastDelta)/timeChange;
        lastDelta = delta;
        lastTime = startTime;
        while(System.nanoTime() - startTime < 10000){

        }
        return kp * delta + ki * deltaSum + kd * dDelta;
    }
}
