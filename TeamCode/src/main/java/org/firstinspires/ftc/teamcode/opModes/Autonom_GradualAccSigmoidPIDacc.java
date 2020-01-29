package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

@Autonomous
@Disabled
public class Autonom_GradualAccSigmoidPIDacc extends LinearOpMode {
    private RevBulkData bulkData;
    private ExpansionHubMotor encoderDreapta, encoderSpate, encoderStanga;
    private ExpansionHubEx expansionHub;
    private long EncSp, EncSt, EncDr;
    private ExpansionHubMotor motorss, motorsf, motords, motordf;

    private long vMax = 5000; // ticks/sec
    private long accMax = 2500; // ticks/sec/sec
    private long tAcc = vMax / accMax; // sec
    private long acc;
    private long t;
    private double tTemp; //sec
    private double d;
    private double v, tStart;
    private boolean gradualAcc, startThreads = true;
    private double currentAcc, currentV, lastV, lastT;
    private double kp = 0.0001, ki = 0, kd = 0;
    private double power = 0, delta = 0;
    private double timeChange, lastTime, lastDelta, deltaSum, dDelta;
    @Override
    public void runOpMode(){
        expansionHub = hardwareMap.get(ExpansionHubEx.class, configs.expansionHubOdometrieName);
        motorss = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, configs.ssName);
        motords = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, configs.dsName);
        motorsf = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, configs.sfName);
        motordf = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, configs.dfName);

        encoderDreapta = motordf;
        encoderSpate = motorsf;
        encoderStanga = motorss;

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
        if (startThreads) {
            Encodere.start();
            PIDVelocity.start();
        }
        while(!isStarted()){
            telemetry.addData("EncDr", EncDr);
            telemetry.addData("EncSt", EncSt);
            telemetry.addData("acc", currentAcc);
            telemetry.update();
        }
        waitForStart();
        gradualAcc(20000);
        sleep(2000);
    }
    private Thread Encodere = new Thread(new Runnable() {
        long encSpVal, encDrVal, encStVal;
        double T, V;
        @Override
        public void run() {
            while (startThreads) {
                bulkData = expansionHub.getBulkInputData();
                encSpVal = bulkData.getMotorVelocity(encoderSpate);
                encDrVal = bulkData.getMotorVelocity(encoderDreapta);
                encStVal = bulkData.getMotorVelocity(encoderStanga);
                EncSp = -encSpVal;
                EncSt = encStVal;
                EncDr = encDrVal;

                T = System.currentTimeMillis();
                V = (EncSp+EncDr)/2.0;

                currentAcc = (V - lastV)/((T - lastT)/1000.0);
                currentV = V;

                lastT = T;
                lastV = V;
            }
        }
    });
    public void gradualAcc(long distantaEnc) {
        t = distantaEnc / vMax + tAcc; // sec
        tStart = System.currentTimeMillis() / 1000.0; // sec
        if (vMax > Math.sqrt(distantaEnc * accMax)) {
            vMax = (long) Math.sqrt(distantaEnc * accMax);
            tAcc = vMax / accMax;
            t = distantaEnc / vMax + tAcc;
        }

        gradualAcc = true;
        while ((tTemp = System.currentTimeMillis() / 1000.0 - tStart) < tAcc) {
            v = tTemp * accMax;
            d = (tTemp * v) / 2;
            acc = accMax;
            telemetry.addData("1: -_-", "o_o");
            telemetry.addData("D: ", d);
            telemetry.addData("Ttemp: ", tTemp);
            telemetry.addData("V: ", v);
            telemetry.addData("Power", power);
            telemetry.addData("delta", delta);
            telemetry.addData("currentAcc",currentAcc);
            telemetry.addData("currentV",currentV);
            telemetry.update();
        }
        v = vMax;

        while ((tTemp = (System.currentTimeMillis() / 1000.0 - tStart)) < t - tAcc) {
            acc = 0;
            d = vMax * (2 * tTemp - tAcc) / 2;
            telemetry.addData("2: -_-", "o_o");
            telemetry.addData("D: ", d);
            telemetry.addData("Ttemp: ", tTemp);
            telemetry.addData("V: ", v);
            telemetry.addData("Power", power);
            telemetry.addData("delta", delta);
            telemetry.addData("currentAcc",currentAcc);
            telemetry.addData("currentV",currentV);
            telemetry.update();
        }
        while ((tTemp = System.currentTimeMillis() / 1000.0 - tStart) < t) {
            acc = -accMax;
            v = (int) (vMax - (tTemp - t + tAcc) * accMax);
            d = vMax * (2 * t - 3 * tAcc) / 2.0 + (tAcc + tTemp - t) * (v + vMax) / 2.0;
            telemetry.addData("3: -_-", "o_o");
            telemetry.addData("D: ", d);
            telemetry.addData("Ttemp: ", tTemp);
            telemetry.addData("V: ", v);
            telemetry.addData("Power:", power);
            telemetry.addData("delta", delta);
            telemetry.addData("currentAcc",currentAcc);
            telemetry.addData("currentV",currentV);
            telemetry.update();
        }
        power(0,0,0,0);
    }
    private Thread PIDVelocity = new Thread(new Runnable() {
        @Override
        public void run() {
            while (startThreads) {
                if (gradualAcc) {
                    if(acc != 0){
                        delta = acc - currentAcc;
                        power = power + PID(acc - currentAcc, ki, kp, kd);
                    }
                    else{
                        delta = v - currentV;
                        power = power + PID(v - currentV, ki, kp, kd);
                    }
                    if(power > 1 || power < -1){
                        power = Math.signum(power) * 1;
                    }
                    power(-power, -power, -power, -power);
                }
            }
        }
    });
    public double PID(double delta, double kp, double ki, double kd){
        timeChange = System.currentTimeMillis() - lastTime;
        deltaSum += (delta * timeChange);
        dDelta = (delta - lastDelta);
        lastDelta = delta;
        lastTime = System.currentTimeMillis();
        return kp * delta + ki * deltaSum + kd * dDelta;
    }
    private void power(double ds, double df, double ss, double sf){
        motordf.setPower(df);
        motorss.setPower(ss);
        motorsf.setPower(sf);
        motords.setPower(ds);
    }
}
