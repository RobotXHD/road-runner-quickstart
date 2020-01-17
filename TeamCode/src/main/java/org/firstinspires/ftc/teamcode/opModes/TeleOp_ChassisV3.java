package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

@TeleOp
@Disabled
public class TeleOp_ChassisV3 extends OpMode {

    private RevBulkData bulkData;
    private ExpansionHubMotor encoderDreapta, encoderSpate,encoderStanga;
    private ExpansionHubEx expansionHub;
    private ExpansionHubMotor motordf;
    private ExpansionHubMotor motords;
    private ExpansionHubMotor motorsf;
    private ExpansionHubMotor motorss;
    private double EncSp, EncSt, EncDr;
    private double targetVx, targetVy, targetRot;
    private double errorSt, errorDr, errorRot, errorSideways;
    private double df, ds, sf, ss;
    private double k = 0.00001;
    private double kpf = -10*k, kps = -10*k, kpr= -10*k;
    private double systime;
    private double max;
    private boolean stop = false;

    private Thread Chassis = new Thread(new Runnable() {
        @Override
        public void run() {
            while(!stop){
                targetVx = gamepad1.left_stick_x*20000;
                targetVy = -gamepad1.left_stick_y*20000;
                targetRot = gamepad1.right_stick_x*20000;

                errorSt = targetVy- EncSt;
                errorDr = targetVy - EncDr;
                errorSideways = targetVx*1.8 - EncSp;
                errorRot = targetRot - ((EncDr - EncSt)/2);

                if(Math.abs(errorDr) < 50){
                    errorDr = 0;
                }
                if(Math.abs(errorSt) < 50){
                    errorSt = 0;
                }
                if(Math.abs(errorRot) < 100){
                    errorRot = 0;
                }
                if(Math.abs(errorSideways) < 100){
                    errorSideways = 0;
                }

                df = kpf * errorDr - kpr * errorRot;
                ds = kpf * errorDr - kpr * errorRot;
                sf = kpf * errorSt + kpr * errorRot;
                ss = kpf * errorSt + kpr * errorRot;

                max = Math.abs(df);
                if(Math.abs(ds) > max) max = Math.abs(ds);
                if(Math.abs(sf) > max) max = Math.abs(sf);
                if(Math.abs(ss) > max) max = Math.abs(ss);

                if(max > 1){
                    df /= max;
                    ds /= max;
                    sf /= max;
                    ss /= max;
                }

                motordf.setPower(df);
                motorss.setPower(ss);
                motords.setPower(ds);
                motorsf.setPower(sf);


                if(gamepad1.a && (systime + 200 < System.currentTimeMillis())){
                    kpf+=k;
                    systime = System.currentTimeMillis();
                }
                else if(gamepad1.b && (systime +200 < System.currentTimeMillis())){
                    kpf-=k;
                    systime = System.currentTimeMillis();
                }
            }
        }
    });

    private Thread Encoders = new Thread(new Runnable() {
        long encSpVal, encDrVal, encStVal;
        @Override
        public void run() {
            while (!stop) {
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

    @Override
    public void init() {
        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub Odometrie");

        motordf = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, "df");
        motords = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, "ds");
        motorsf = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, "sf");
        motorss = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, "ss");

        encoderDreapta = (ExpansionHubMotor) hardwareMap.get(DcMotor.class,"encoderDreapta");
        encoderSpate = (ExpansionHubMotor) hardwareMap.get(DcMotor.class,"encoderSpate");
        encoderStanga =(ExpansionHubMotor) hardwareMap.get(DcMotor.class,"encoderStanga");

        motorsf.setDirection(DcMotorSimple.Direction.REVERSE);
        motorss.setDirection(DcMotorSimple.Direction.REVERSE);

        motorss.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorsf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motordf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motords.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motordf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motords.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorsf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorss.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        systime = System.currentTimeMillis();

        Chassis.start();
        Encoders.start();
    }

    @Override
    public void loop() {
        telemetry.addData("EncDr", EncDr);
        telemetry.addData("EncSt", EncSt);
        telemetry.addData("EncSp", EncSp);
      telemetry.addData("Y: ",-gamepad1.left_stick_y);
        telemetry.addData("ErrorDr: ",errorDr * kpf);
        telemetry.addData("ErrorSt: ",errorSt * kpf);
        //telemetry.addData("ErrorSp: ",errorSideways);
        telemetry.addData("ErrorRot: ",errorRot * kpr);
        telemetry.addData("Ds: ",ds);
        telemetry.addData("Df: ",df);
        telemetry.addData("Ss: ",ss);
        telemetry.addData("Sf: ",sf);
        telemetry.addData("Kpf: ",kpf);
        telemetry.update();
    }

    @Override
    public void stop() {
        stop = true;
    }
}