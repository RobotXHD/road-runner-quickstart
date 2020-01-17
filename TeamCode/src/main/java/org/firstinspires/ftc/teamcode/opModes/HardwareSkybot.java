package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;


public class HardwareSkybot extends LinearOpMode {
    private RevBulkData bulkData;
    private ExpansionHubMotor encoderDreapta, encoderSpate,encoderStanga;
    private ExpansionHubEx expansionHub;
    long EncSp, EncSt, EncDr;
    public DcMotorEx motorss, motorsf, motords, motordf;

    public Servo servoRot,servoSj,servoClamp;
    public HardwareMap hwmap;
    public boolean startThreads = false ;

    public boolean gradualAcc;
    public volatile long v = 0;
    public double power = 0;
    public double d;
    public double kp = -0.01;
    public double delta;

    public HardwareSkybot(boolean startThreads){this.startThreads = startThreads;}

    public void init(HardwareMap hmap){
        hwmap = hmap;
        expansionHub = hwmap.get(ExpansionHubEx.class, "Expansion Hub Odometrie");
        motorss = hwmap.get(DcMotorEx.class, "ss");
        motords = hwmap.get(DcMotorEx.class, "ds");
        motorsf = hwmap.get(DcMotorEx.class, "sf");
        motordf = hwmap.get(DcMotorEx.class, "df ");

        encoderDreapta = (ExpansionHubMotor)hwmap.get(DcMotor.class, "encoderDreapta");
        encoderSpate = (ExpansionHubMotor) hwmap.get(DcMotor.class, "encoderSpate");
        encoderStanga = (ExpansionHubMotor) hwmap.get(DcMotor.class, "encoderStanga");

        servoRot = hwmap.get(Servo.class, "rot");
        servoSj = hwmap.get(Servo.class, "sj");
        servoClamp = hwmap.get(Servo.class, "clamp");

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

        servoRot.setPosition(configs.pozitie_servoRot_minim);
        servoSj.setPosition(configs.pozitie_servorSj_minim);

        if(startThreads){
            Encodere.start();
            PIDVelocity.start();
        }
    }

    public void gradualAcc(long distantaEnc){
        long vMax = 5000; // ticks/sec
        long accMax = 2500; // ticks/sec/sec
        long tAcc = vMax / accMax; // sec
        long t = distantaEnc/vMax + tAcc; // sec
        double tTemp; //sec
        if(vMax > Math.sqrt(distantaEnc * accMax)){
            vMax = (long) Math.sqrt(distantaEnc * accMax);
            tAcc = vMax / accMax;
            t = distantaEnc/vMax + tAcc;
        }
        gradualAcc = true;
        double tStart = System.currentTimeMillis()/1000.0; // sec
        while((tTemp = System.currentTimeMillis()/1000.0 - tStart) < tAcc){
            v = (int) tTemp * accMax;
            d = (tTemp*v)/2;
            telemetry.addData("D: ",d);
            telemetry.addData("Ttemp: ",tTemp);
            telemetry.addData("V: ",v);
            telemetry.update();
        }
        v = vMax;
        while((tTemp = (System.currentTimeMillis() - tStart)/1000.0) < t - tAcc){
            d = vMax*(2*tTemp- tAcc)/2;
            telemetry.addData("D: ",d);
            telemetry.addData("Ttemp: ",tTemp);
            telemetry.addData("V: ",v);
            telemetry.update();
        }
        while((tTemp = (System.currentTimeMillis() - tStart)/1000.0) < t){
            v = (int) (vMax - (tTemp - t + tAcc) * accMax);
            d = (tAcc+tTemp-t)*(v+vMax)/2;
            telemetry.addData("D: ",d);
            telemetry.addData("Ttemp: ",tTemp);
            telemetry.addData("V: ",v);
            telemetry.update();
        }
        power(0,0,0,0);
        gradualAcc = false;
    }

    public void power(double df, double ss, double sf, double ds) {
        //++--
        motordf.setPower(df);
        motorss.setPower(ss);
        motorsf.setPower(sf);
        motords.setPower(ds);
    }

    public void stop_and_reset() {
        motordf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motords.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorsf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorss.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motordf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motords.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorsf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorss.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private Thread Encodere = new Thread(new Runnable() {
        long encSpVal, encDrVal, encStVal;
        @Override
        public void run() {
            while (startThreads) {
                bulkData = expansionHub.getBulkInputData();
                encSpVal = bulkData.getMotorVelocity(encoderSpate);
                encDrVal = bulkData.getMotorVelocity(encoderDreapta);
                encStVal = bulkData.getMotorVelocity(encoderStanga);
                EncSp = -encSpVal;
                EncSt = -encStVal;
                EncDr = encDrVal;
            }
        }
    });
    private Thread PIDVelocity = new Thread(new Runnable() {
        @Override
        public void run() {
            while(startThreads){
                if(gradualAcc){
                    delta = v - ((EncSt + EncDr) / 2);
                    power = power + delta * kp;
                }
            }
        }
    });
    @Override
    public void runOpMode(){

    }
}
