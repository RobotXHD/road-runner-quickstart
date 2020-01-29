package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

@Autonomous
@Disabled
public class Test_Formule_miscare_Auto extends LinearOpMode {
    private RevBulkData bulkData;
    private ExpansionHubMotor encoderDreapta, encoderSpate, encoderStanga;
    private ExpansionHubEx expansionHub;
    long EncSp, EncSt, EncDr;
    public DcMotorEx motorss, motorsf, motords, motordf;

    public HardwareMap hwmap;
    long vMax = 5000; // ticks/sec
    long accMax = 2500; // ticks/sec/sec
    long tAcc = vMax / accMax; // sec
    long t;
    double tTemp; //sec
    double d;
    double v, tStart;
    boolean gradualAcc, startThreads = true;
    public long kp = 0, ki = 0, kd = 0;
    public double power = 0, delta = 0;
    double timeChange, lastTime, lastDelta, deltaSum, dDelta;

    @Override
    public void runOpMode() throws InterruptedException {
        hwmap = hardwareMap;
        expansionHub = hwmap.get(ExpansionHubEx.class, configs.expansionHubOdometrieName);
        motorss = hwmap.get(DcMotorEx.class, "ss");
        motords = hwmap.get(DcMotorEx.class, "ds");
        motorsf = hwmap.get(DcMotorEx.class, "sf");
        motordf = hwmap.get(DcMotorEx.class, "df");

        encoderDreapta = (ExpansionHubMotor) hwmap.get(DcMotor.class, configs.encDrName);
        encoderSpate = (ExpansionHubMotor) hwmap.get(DcMotor.class, configs.encSpName);
        encoderStanga = (ExpansionHubMotor) hwmap.get(DcMotor.class, configs.colectStName);

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
        motorss.setDirection(DcMotorSimple.Direction.REVERSE);
        if (startThreads) {
            Encodere.start();
            //PIDVelocity.start();
        }

        waitForStart();

        telemetry.addData("tacc", tAcc);
        telemetry.addData("ttemp", tTemp);
        telemetry.addData("tstart", tStart);
        telemetry.addData("t", t);
        telemetry.addData("d", d);
        telemetry.update();
        sleep(5000);
        startThreads = false;
    }

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
            telemetry.addData("1: -_-", "o_o");
            telemetry.addData("D: ", d);
            telemetry.addData("Ttemp: ", tTemp);
            telemetry.addData("V: ", v);
            telemetry.update();
        }
        v = vMax;

        while ((tTemp = (System.currentTimeMillis() / 1000.0 - tStart)) < t - tAcc) {
            d = vMax * (2 * tTemp - tAcc) / 2;
            telemetry.addData("2: -_-", "o_o");
            telemetry.addData("D: ", d);
            telemetry.addData("Ttemp: ", tTemp);
            telemetry.addData("V: ", v);
            telemetry.update();
        }
        while ((tTemp = System.currentTimeMillis() / 1000.0 - tStart) < t) {
            v = (int) (vMax - (tTemp - t + tAcc) * accMax);
            d = vMax * (2 * t - 3 * tAcc) / 2.0 + (tAcc + tTemp - t) * (v + vMax) / 2.0;
            telemetry.addData("3: -_-", "o_o");
            telemetry.addData("D: ", d);
            telemetry.addData("Ttemp: ", tTemp);
            telemetry.addData("V: ", v);
            telemetry.update();
        }
        gradualAcc = false;
    }

    private Thread Encodere = new Thread(new Runnable() {
        long encSpVal, encDrVal, encStVal;
        @Override
        public void run() {
            while (startThreads) {
                bulkData = expansionHub.getBulkInputData();
                encSpVal = bulkData.getMotorCurrentPosition(encoderSpate);
                encDrVal = bulkData.getMotorCurrentPosition(encoderDreapta);
                encStVal = bulkData.getMotorCurrentPosition(encoderStanga);
                EncSp = -encSpVal;
                EncSt = -encStVal;
                EncDr = encDrVal;
            }
        }
    });
    /*
    private Thread PIDVelocity = new Thread(new Runnable() {
        @Override
        public void run() {
            while (startThreads) {
                if (gradualAcc) {
                    delta = d - ((EncSt + EncDr) / 2.0);
                    power = power + PID(delta, kp, ki, kd);
                }
            }
        }
    });
    public void power(double df, double ss, double sf, double ds) {
        //++--
        motordf.setPower(df);
        motorss.setPower(ss);
        motorsf.setPower(sf);
        motords.setPower(ds);
    }
    public double PID(double delta, double kp, double ki, double kd){
        timeChange = System.currentTimeMillis() - lastTime;
        deltaSum += (delta * timeChange);
        dDelta = (delta - lastDelta);
        lastDelta = delta;
        lastTime = System.currentTimeMillis();
        return kp * delta + ki * deltaSum + kd * dDelta;
    }*/
}
