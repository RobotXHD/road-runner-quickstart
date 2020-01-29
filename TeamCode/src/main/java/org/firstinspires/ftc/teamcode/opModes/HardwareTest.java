package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

@Autonomous
@Disabled
public class HardwareTest extends LinearOpMode {
    public double v;
    public boolean gradualAcc;
    public double encSp, encSt, encDr;
    public  RevBulkData bulkData;
    public ExpansionHubMotor encoderSpate, encoderDreapta, encoderStanga;
    public ExpansionHubEx hubEx;
    public DcMotorEx motorss, motorsf, motords, motordf;
    double delta, kp = 0.01, p = 0;
    final double kV = 0.054;

    @Override
    public void runOpMode() throws InterruptedException {
        hubEx = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub Odometrie");
        
        encoderDreapta = (ExpansionHubMotor) hardwareMap.get(DcMotor.class, "encoderDreapta");
        encoderSpate = (ExpansionHubMotor) hardwareMap.get(DcMotor.class, "encoderSpate");
        encoderStanga = (ExpansionHubMotor) hardwareMap.get(DcMotor.class, "encoderStanga");

        motorss = hardwareMap.get(DcMotorEx.class, "ss");
        motords = hardwareMap.get(DcMotorEx.class, "ds");
        motorsf = hardwareMap.get(DcMotorEx.class, "sf");
        motordf = hardwareMap.get(DcMotorEx.class, "df ");

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

        motordf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motords.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorsf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorss.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motords.setDirection(DcMotorSimple.Direction.REVERSE);
        motorss.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        Encodere.start();
        pidV.start();
        gradualAcc(20000);
    }
    public void gradualAcc(long distantaEnc){
        double vMax = 5000; // ticks/sec
        double accMax = 500; // ticks/sec/sec
        double tAcc = vMax / accMax; // sec
        double t = distantaEnc/vMax + tAcc; // sec
        double tTemp; //sec
        if(vMax > Math.sqrt(distantaEnc * accMax)){
            vMax = Math.sqrt(distantaEnc * accMax);
            tAcc = vMax / accMax;
            t = distantaEnc/vMax + tAcc;
        }
        gradualAcc = true;
        double tStart = System.currentTimeMillis(); // milisec
        while((tTemp = (System.currentTimeMillis() - tStart)/1000.0) < tAcc){
            v = tTemp * accMax;
            telemetry.addData("Tacc", tAcc);
            telemetry.addData("V", v);
            telemetry.addData("t", t);
            telemetry.addData("delta", delta);
            telemetry.addData("p", p);
            telemetry.update();
        }
        v = vMax;
        while(((System.currentTimeMillis() - tStart)/1000.0) < (t - tAcc)){
            telemetry.addData("Tacc", tAcc);
            telemetry.addData("V", v);
            telemetry.addData("t", t);
            telemetry.addData("delta", delta);
            telemetry.addData("p", p);
            telemetry.addData("stage 2", "");
            telemetry.update();
        }
        while((tTemp = (System.currentTimeMillis() - tStart)/1000.0) < t){
            v = (vMax - (tTemp - t + tAcc) * accMax);
            telemetry.addData("Tacc", tAcc);
            telemetry.addData("V", v);
            telemetry.addData("t", t);
            telemetry.addData("delta", delta);
            telemetry.addData("p", p);
            telemetry.addData("stage 3", "");
            telemetry.update();
        }
        gradualAcc = false;
    }

    public Thread pidV = new Thread(new Runnable() {
        @Override
        public void run() {
            while(!isStopRequested()){
                delta = v - ((encSt + encDr)/2.0);
                p = 0.054 * v + delta * kp;
                if(Math.abs(p) > 2700){
                    p = Math.signum(p) * 2700;
                }
                velocity(p, -p, -p, p);
            }
        }
    });
    private Thread Encodere = new Thread(new Runnable() {
        long encSpVal, encDrVal, encStVal;
        @Override
        public void run() {
            while (!isStopRequested()) {
                bulkData = hubEx.getBulkInputData();
                encSpVal = bulkData.getMotorVelocity(encoderSpate);
                encDrVal = bulkData.getMotorVelocity(encoderDreapta);
                encStVal = bulkData.getMotorVelocity(encoderStanga);
                encSp = -encSpVal;
                encSt = -encStVal;
                encDr = encDrVal;
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
    public void velocity(double df, double ss, double sf, double ds){
        motordf.setVelocity(df);
        motorss.setVelocity(ss);
        motorsf.setVelocity(sf);
        motords.setVelocity(ds);
    }
}
