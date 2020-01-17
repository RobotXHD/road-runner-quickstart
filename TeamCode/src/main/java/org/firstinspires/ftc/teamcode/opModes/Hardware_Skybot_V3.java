package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

public class  Hardware_Skybot_V3 extends LinearOpMode {

    public RevBulkData bulkData;
    public ExpansionHubMotor encoderDreapta, encoderSpate, encoderStanga;
    public ExpansionHubEx expansionHub;
    public ExpansionHubMotor motorss, motorsf, motords, motordf;
    public DcMotor motorColectSt, motorColectDr;
    public Servo  servoPlatformaSt, servoPlatformaDr, servoCapstone, servoParcare;
    public int verifications = 0;
    public int totalY = 0, totalX = 0, totalRot = 0;
    public double correctionR,correctionY,correctionX;
    public double ds, df, ss, sf, Y, tempRot, max;
    public double encDr, encSt, encSp;
    public double Rotatie = 0, ticksPerDegree = PIDControllerTestConfig.rotationCalib;
    public PIDControllerAdevarat pidRotatie = new PIDControllerAdevarat(0,0,0);
    public PIDControllerAdevarat pidY = new PIDControllerAdevarat(0,0,0);
    public PIDControllerAdevarat pidX = new PIDControllerAdevarat(0,0,0);
    public boolean startTh = false;

    public Hardware_Skybot_V3(boolean startThreads) {startTh = startThreads;}
    public Hardware_Skybot_V3(){}

    public void Init(HardwareMap hard){
        expansionHub = hard.get(ExpansionHubEx.class, configs.expansionHubOdometrieName);
        motorss = (ExpansionHubMotor) hard.get(DcMotorEx.class, configs.ssName);
        motords = (ExpansionHubMotor) hard.get(DcMotorEx.class, configs.dsName);
        motorsf = (ExpansionHubMotor) hard.get(DcMotorEx.class, configs.sfName);
        motordf = (ExpansionHubMotor) hard.get(DcMotorEx.class, configs.dfName);
        //ss = spate; sf = stanga; df = dreapta
        encoderDreapta = motordf;
        encoderSpate = motorss;
        encoderStanga = motorsf;

        motorColectDr = hard.get(DcMotor.class, configs.colectDrName);
        motorColectSt = hard.get(DcMotor.class, configs.colectStName);

        servoPlatformaDr = hard.servo.get(configs.servoPlatformaDrName);
        servoPlatformaSt = hard.servo.get(configs.servoPlatformaStName);
        servoCapstone = hard.servo.get(configs.servoCapstoneName);
        servoParcare = hard.servo.get(configs.servoParcareName);

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

        servoCapstone.setPosition(0);

        if(startTh){
            encoderRead.start();
        }
        pidRotatie.setSetpoint(0);
        pidY.setSetpoint(0);
        pidX.setSetpoint(0);

        pidRotatie.setTolerance(PIDControllerTestConfig.toleranceRotatie);
        pidY.setTolerance(PIDControllerTestConfig.toleranceY);
        pidX.setTolerance(PIDControllerTestConfig.toleranceX);

        pidRotatie.setPID(PIDControllerTestConfig.p, PIDControllerTestConfig.i, PIDControllerTestConfig.d);
        pidY.setPID(PIDControllerTestConfig.py, PIDControllerTestConfig.iy, PIDControllerTestConfig.dy);
        pidX.setPID(PIDControllerTestConfig.px, PIDControllerTestConfig.ix, PIDControllerTestConfig.dx);

        pidRotatie.enable();
        pidY.enable();
        pidX.enable();

    }

    private void powerY(double ds, double df, double ss, double sf) {
        motordf.setPower(df);
        motorsf.setPower(sf);
        motords.setPower(ds);
        motorss.setPower(ss);
    }

    private void powerX(double ds, double df, double ss, double sf) {
        motorss.setPower(ss);
        motorsf.setPower(sf);
        motords.setPower(ds);
        motordf.setPower(df);
    }

    private void powerRot(double ds, double df, double ss, double sf) {
        motordf.setPower(df);
        motorss.setPower(ss);
        motorsf.setPower(sf);
        motords.setPower(ds);
    }

    @Override
    public void runOpMode() {
    }

    public void gotoY(double incrementalY, double maxPow){
        totalY += incrementalY;
        verifications = 0;

        pidY.enable();
        pidX.enable();
        pidRotatie.enable();

        pidX.setSetpoint(totalX);
        pidY.setSetpoint(totalY);
        pidRotatie.setSetpoint(totalRot);

        pidX.setTolerance(PIDControllerTestConfig.toleranceX);
        pidY.setTolerance(PIDControllerTestConfig.toleranceY);
        pidRotatie.setTolerance(PIDControllerTestConfig.toleranceRotatie);
        do{
            Y = (encDr + encSt)/2;
            correctionR = -pidRotatie.performPID(Rotatie);
            correctionY = -pidY.performPID(Y);
            correctionX = pidX.performPID(encSp);
            ds = correctionR + correctionY - correctionX;
            df = correctionR + correctionY + correctionX;
            ss = -correctionR + correctionY + correctionX;
            sf = -correctionR + correctionY - correctionX;

            max = Math.abs(ds);
            max = Math.abs(df) > max ? Math.abs(df) : max;
            max = Math.abs(sf) > max ? Math.abs(sf) : max;
            max = Math.abs(ss) > max ? Math.abs(ss) : max;

            if (max > maxPow) {
                ds /= max;
                df /= max;
                sf /= max;
                ss /= max;
            }
            powerY(ds, df, ss, sf);
            verifications = pidY.onTarget() ? verifications + 1 : 0;
        }while(verifications < PIDControllerTestConfig.targetVerifications);
        powerY(0,0,0,0);
    }

    public void gotoY(double incrementalY, double maxPow, double tolerance){
        totalY += incrementalY;
        verifications = 0;

        pidY.enable();
        pidX.enable();
        pidRotatie.enable();

        pidX.setSetpoint(totalX);
        pidY.setSetpoint(totalY);
        pidRotatie.setSetpoint(totalRot);

        pidX.setTolerance(PIDControllerTestConfig.toleranceX);
        pidY.setTolerance(tolerance);
        pidRotatie.setTolerance(PIDControllerTestConfig.toleranceRotatie);
        do{
            Y = (encDr + encSt)/2;
            correctionR = -pidRotatie.performPID(Rotatie);
            correctionY = -pidY.performPID(Y);
            correctionX = pidX.performPID(encSp);
            ds = correctionR + correctionY - correctionX;
            df = correctionR + correctionY + correctionX;
            ss = -correctionR + correctionY + correctionX;
            sf = -correctionR + correctionY - correctionX;

            max = Math.abs(ds);
            max = Math.abs(df) > max ? Math.abs(df) : max;
            max = Math.abs(sf) > max ? Math.abs(sf) : max;
            max = Math.abs(ss) > max ? Math.abs(ss) : max;

            if (max > maxPow) {
                ds /= max;
                df /= max;
                sf /= max;
                ss /= max;
            }
            powerY(ds, df, ss, sf);
            verifications = pidY.onTarget() ? verifications + 1 : 0;
        }while(verifications < PIDControllerTestConfig.targetVerifications);
        powerY(0,0,0,0);
    }



    public void rotatie(double incrementalRot, double maxPow){
        totalRot+=incrementalRot;
        verifications = 0;

        pidY.enable();
        pidX.enable();
        pidRotatie.enable();

        pidX.setSetpoint(totalX);
        pidY.setSetpoint(totalY);
        pidRotatie.setSetpoint(totalRot);

        pidX.setTolerance(PIDControllerTestConfig.toleranceX);
        pidY.setTolerance(PIDControllerTestConfig.toleranceY);
        pidRotatie.setTolerance(PIDControllerTestConfig.toleranceRotatie);
        do{
            Y = (encDr + encSt)/2;
            correctionR = -pidRotatie.performPID(Rotatie);
            correctionY = -pidY.performPID(Y);
            correctionX = pidX.performPID(encSp);
            ds = correctionR + correctionY - correctionX;
            df = correctionR + correctionY + correctionX;
            ss = -correctionR + correctionY + correctionX;
            sf = -correctionR + correctionY - correctionX;

            max = Math.abs(ds);
            max = Math.abs(df) > max ? Math.abs(df):max;
            max = Math.abs(sf) > max ? Math.abs(sf):max;
            max = Math.abs(ss) > max ? Math.abs(ss):max;
            if(max > maxPow){
                ds /= max;
                df /= max;
                sf /= max;
                ss /= max;
            }
            powerRot(ds, df, ss, sf);
            verifications = pidRotatie.onTarget() ? verifications+1 : 0;
        }while (verifications < PIDControllerTestConfig.targetVerifications);
        powerRot(0,0,0,0);
    }

    public void rotatie(double incrementalRot, double maxPow, double tolerance){
        totalRot+=incrementalRot;
        verifications = 0;

        pidY.enable();
        pidX.enable();
        pidRotatie.enable();

        pidX.setSetpoint(totalX);
        pidY.setSetpoint(totalY);
        pidRotatie.setSetpoint(totalRot);

        pidX.setTolerance(PIDControllerTestConfig.toleranceX);
        pidY.setTolerance(PIDControllerTestConfig.toleranceY);
        pidRotatie.setTolerance(tolerance);
        do{
            Y = (encDr + encSt)/2;
            correctionR = -pidRotatie.performPID(Rotatie);
            correctionY = -pidY.performPID(Y);
            correctionX = pidX.performPID(encSp);
            ds = correctionR + correctionY - correctionX;
            df = correctionR + correctionY + correctionX;
            ss = -correctionR + correctionY + correctionX;
            sf = -correctionR + correctionY - correctionX;

            max = Math.abs(ds);
            max = Math.abs(df) > max ? Math.abs(df):max;
            max = Math.abs(sf) > max ? Math.abs(sf):max;
            max = Math.abs(ss) > max ? Math.abs(ss):max;
            if(max > maxPow){
                ds /= max;
                df /= max;
                sf /= max;
                ss /= max;
            }
            powerRot(ds, df, ss, sf);
            verifications = pidRotatie.onTarget() ? verifications+1 : 0;
        }while (verifications < PIDControllerTestConfig.targetVerifications);
        powerRot(0,0,0,0);
    }



    public void gotoX(double incrementalX, double maxPow){
        totalX += incrementalX;
        verifications = 0;

        pidY.enable();
        pidX.enable();
        pidRotatie.enable();

        pidX.setSetpoint(totalX);
        pidY.setSetpoint(totalY);
        pidRotatie.setSetpoint(totalRot);

        pidX.setTolerance(PIDControllerTestConfig.toleranceX);
        pidY.setTolerance(PIDControllerTestConfig.toleranceY);
        pidRotatie.setTolerance(PIDControllerTestConfig.toleranceRotatie);
        do{
            Y = (encDr + encSt)/2;
            correctionR = -pidRotatie.performPID(Rotatie);
            correctionY = -pidY.performPID(Y);
            correctionX = pidX.performPID(encSp);
            ds = correctionR + correctionY - correctionX;
            df = correctionR + correctionY + correctionX;
            ss = -correctionR + correctionY + correctionX;
            sf = -correctionR + correctionY - correctionX;

            max = Math.abs(ds);
            max = Math.abs(df) > max ? Math.abs(df):max;
            max = Math.abs(sf) > max ? Math.abs(sf):max;
            max = Math.abs(ss) > max ? Math.abs(ss):max;
            if(max > maxPow){
                ds /= max;
                df /= max;
                sf /= max;
                ss /= max;
            }
            powerX(ds, df, ss, sf);
            verifications = pidX.onTarget() ? verifications+1 : 0;
        }while(verifications < PIDControllerTestConfig.targetVerifications);
        powerX(0,0,0,0);
    }

    public void gotoX(double incrementalX, double maxPow, double tolerance){
        totalX += incrementalX;
        verifications = 0;

        pidY.enable();
        pidX.enable();
        pidRotatie.enable();

        pidX.setSetpoint(totalX);
        pidY.setSetpoint(totalY);
        pidRotatie.setSetpoint(totalRot);

        pidX.setTolerance(tolerance);
        pidY.setTolerance(PIDControllerTestConfig.toleranceY);
        pidRotatie.setTolerance(PIDControllerTestConfig.toleranceRotatie);
        do{
            Y = (encDr + encSt)/2;
            correctionR = -pidRotatie.performPID(Rotatie);
            correctionY = -pidY.performPID(Y);
            correctionX = pidX.performPID(encSp);
            ds = correctionR + correctionY - correctionX;
            df = correctionR + correctionY + correctionX;
            ss = -correctionR + correctionY + correctionX;
            sf = -correctionR + correctionY - correctionX;

            max = Math.abs(ds);
            max = Math.abs(df) > max ? Math.abs(df):max;
            max = Math.abs(sf) > max ? Math.abs(sf):max;
            max = Math.abs(ss) > max ? Math.abs(ss):max;
            if(max > maxPow){
                ds /= max;
                df /= max;
                sf /= max;
                ss /= max;
            }
            powerX(ds, df, ss, sf);
            verifications = pidX.onTarget() ? verifications+1 : 0;
        }while(verifications < PIDControllerTestConfig.targetVerifications);
        powerX(0,0,0,0);
    }




    public void alinierePlaca(double incrementalY, double power){
        pidY.disable();
        pidX.enable();
        pidRotatie.enable();
        totalY+=incrementalY;
        do{
            Y = (encDr + encSt)/2;
            correctionR = -pidRotatie.performPID(Rotatie);
            correctionX = pidX.performPID(encSp);
            ds = correctionR + power - correctionX;
            df = correctionR + power + correctionX;
            ss = -correctionR + power + correctionX;
            sf = -correctionR + power - correctionX;

            max = Math.abs(ds);
            max = Math.abs(df) > max ? Math.abs(df):max;
            max = Math.abs(sf) > max ? Math.abs(sf):max;
            max = Math.abs(ss) > max ? Math.abs(ss):max;
            if(max > 1){
                ds /= max;
                df /= max;
                sf /= max;
                ss /= max;
            }
            powerY(ds, df, ss, sf);
        }while(Y > totalY + 250 || Y < totalY - 250);
        powerY(0,0,0,0);
    }

    public void rotatiePlaca(double rotatie, int power){
        totalRot+=rotatie;

        pidY.enable();
        pidX.enable();
        pidRotatie.disable();

        do{
            Y = (encDr + encSt)/2;
            correctionY = -pidY.performPID(Y);
            correctionX = pidX.performPID(encSp);
            ds = power + correctionY - correctionX;
            df = power + correctionY + correctionX;
            ss = -power + correctionY + correctionX;
            sf = -power + correctionY - correctionX;

            max = Math.abs(ds);
            max = Math.abs(df) > max ? Math.abs(df):max;
            max = Math.abs(sf) > max ? Math.abs(sf):max;
            max = Math.abs(ss) > max ? Math.abs(ss):max;
            if(max > 1){
                ds /= max;
                df /= max;
                sf /= max;
                ss /= max;
            }
            powerRot(ds, df, ss, sf);
        }while(Rotatie > totalRot + 2 || Rotatie < totalRot - 2);
        powerRot(0,0,0,0);
    }

    private Thread encoderRead = new Thread(new Runnable() {
        long st, dr, sp;
        @Override
        public void run() {
            while (startTh) {
                bulkData = expansionHub.getBulkInputData();
                st = -bulkData.getMotorCurrentPosition(encoderStanga);
                sp = bulkData.getMotorCurrentPosition(encoderSpate);
                dr = -bulkData.getMotorCurrentPosition(encoderDreapta);
                tempRot = ((dr - st)/2.0);
                Rotatie = tempRot/ticksPerDegree;

                encDr = dr - tempRot;
                encSp = sp - Rotatie * PIDControllerTestConfig.sidewaysCalib;
                encSt = st + tempRot;
            }
        }
    });
    public void startColect(){
        motorColectDr.setPower(1);
        motorColectSt.setPower(-1);
    }
    public void stopColect(){
        motorColectDr.setPower(0);
        motorColectSt.setPower(0);
    }
    public void startColectReverse(){
        motorColectDr.setPower(-1);
        motorColectSt.setPower(1);
    }
    public void Colect(double power){
        motorColectDr.setPower(power);
        motorColectSt.setPower(-power);
    }

    public void prindrePlate(){
        servoPlatformaDr.setPosition(0);
        servoPlatformaSt.setPosition(1);
    }

    public void desprindrePlate(){
        servoPlatformaDr.setPosition(1);
        servoPlatformaSt.setPosition(0);
    }

}
