package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

@Autonomous
@Disabled
public class pidCalib extends OpMode {

    private RevBulkData bulkData;
    private ExpansionHubMotor encoderDreapta, encoderSpate, encoderStanga;
    private ExpansionHubEx expansionHub;

    private long EncSp, EncSt, EncDr;

    ExpansionHubMotor motordf, motords, motorsf, motorss;

    private long vMax = 5000; // ticks/sec
    private long accMax = 2500; // ticks/sec/sec
    private long tAcc = vMax / accMax; // sec
    private long t;
    private double tTemp; //sec
    private double d = 0, targetD;
    private double v, tStart;
    private boolean gradualAcc, startThreads = true;
    private double kp = 0.00001, ki = 0.00001, kd = -0.00001;
    private double power = 0, delta = 0;
    private double timeChange, lastTime, lastDelta, deltaSum, dDelta, tempo;
    @Override
    public void init() {
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
        Encodere.start();
        PIDVelocity.start();
        startThreads = true;
        gradualAcc = true;

        tempo = System.currentTimeMillis();
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
                EncDr = -encDrVal;
            }
        }
    });

    private Thread PIDVelocity = new Thread(new Runnable() {
        @Override
        public void run() {
            while (startThreads) {
                if (gradualAcc) {
                    delta = d - ((EncSt + EncDr) / 2.0);
                    power = PID(delta, kp, ki, kd);
                    if(power > 1 || power < -1){
                        power = Math.signum(power) * 1;
                    }
                    power(power, power, power, power);
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

    @Override
    public void loop() {
        if(gamepad1.a && (System.currentTimeMillis() > 200 + tempo)){
            kp *= 2;
            tempo = System.currentTimeMillis();
        }
        else if(gamepad1.b && (System.currentTimeMillis() > 200 + tempo)){
            kp /= 2;
            tempo = System.currentTimeMillis();
        }

        if(gamepad1.x && (System.currentTimeMillis() > 200 + tempo)){
            kd *= 2;
            tempo = System.currentTimeMillis();
        }
        else if(gamepad1.y && (System.currentTimeMillis() > 200 + tempo)){
            kd /= 2;
            tempo = System.currentTimeMillis();
        }
        if(gamepad1.right_bumper){
            kp = 0.01*kd;

        }
        if(gamepad1.left_bumper){
            ki = 0.01*kp;
        }

        if(gamepad1.dpad_up && (System.currentTimeMillis() > 200 + tempo)){
            ki *= 2;
            tempo = System.currentTimeMillis();
        }
        else if(gamepad1.dpad_down && (System.currentTimeMillis() > 200 + tempo)){
            ki /= 2;
            tempo = System.currentTimeMillis();
        }
        telemetry.addData(" >_<", "/*_*/");
        telemetry.addData("D: ", d);
        telemetry.addData("Ttemp: ", tTemp);
        telemetry.addData("V: ", v);
        telemetry.addData("Power:", power);
        telemetry.addData("delta", delta);
        telemetry.addData("Enc", EncDr);
        telemetry.addData("StartThreads", startThreads);
        telemetry.addData("Kp", kp);
        telemetry.addData("kd", kd);
        telemetry.addData("ki", ki);
        telemetry.update();
    }
}
