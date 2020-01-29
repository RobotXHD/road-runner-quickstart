package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static java.lang.Math.abs;
@TeleOp
@Disabled
public class TeleOp_Cyclone extends OpMode {
    private DcMotorEx motordf;
    private DcMotorEx motorsf;
    private DcMotorEx motords;
    private DcMotorEx motorss;
    private DcMotor motorcolect;
    private DcMotor motorext1;
    private DcMotor motordrop;

    private int v = 2;

    private double df;
    private double sf;
    private double ds;
    private double ss;
    private double max;

    private double forward, right, clockwise;

    private boolean stop, apoz = false, alast = true;

    private long fpsC=0;
    private long fpsCLast;
    private long sysTimeC;

    private Thread Chassis = new Thread( new Runnable() {
        @Override
        public void run() {
            while (!stop) {
                if (gamepad1.right_bumper) {
                    v = 1;
                } else if (gamepad1.left_bumper) {
                    v = 2;
                }

                forward = gamepad1.left_stick_y;
                right = -gamepad1.left_stick_x;
                clockwise = gamepad1.right_stick_x;


                df = forward + clockwise - right;
                ss = forward - clockwise - right;
                sf = -forward + clockwise - right;
                ds = -forward - clockwise - right;

                max = abs(sf);
                if (abs(df) > max) {
                    max = abs(df);
                }
                if (abs(ss) > max) {
                    max = abs(ss);
                }
                if (abs(ds) > max) {
                    max = abs(ds);
                }
                if (max > 1) {
                    sf /= max;
                    df /= max;
                    ss /= max;
                    ds /= max;
                }

                fpsC++;
                if (sysTimeC + 1000 < System.currentTimeMillis()) {
                    fpsCLast = fpsC;
                    fpsC = 0;
                    sysTimeC = System.currentTimeMillis();
                }
                if (v == 1) {
                    POWER(df / 5, sf / 5, ds / 5, ss / 5);
                } else if (v == 2) {
                    POWER(df, sf, ds, ss);
                }
            }
        }
    });
    public Thread cyclone = new Thread(new Runnable() {
        @Override
        public void run() {
            while(!stop) {
                boolean abut = gamepad1.b;
                if (alast != abut) {
                    if (gamepad1.b) {
                        apoz = !apoz;
                        if (apoz) motorcolect.setPower(-1);
                        else motorcolect.setPower(0);
                    }
                    alast = abut;
                } else if (gamepad1.left_trigger != 0) {
                    motorcolect.setPower(1);
                }
                if (gamepad1.dpad_left) {
                    motorext1.setPower(1);
                } else if (gamepad1.dpad_right) {
                    motorext1.setPower(-1);
                } else {
                    motorext1.setPower(0);
                }
                if(gamepad1.x){
                    motordrop.setPower(1);
                }
                else if(gamepad1.y){
                    motordrop.setPower(-1);
                }
                else{
                    motordrop.setPower(0);
                }
            }
        }
    });


    @Override
    public void init() {
        motordf = hardwareMap.get(DcMotorEx.class, "motordf");
        motords = hardwareMap.get(DcMotorEx.class, "motords");
        motorsf = hardwareMap.get(DcMotorEx.class, "motorsf");
        motorss = hardwareMap.get(DcMotorEx.class, "motorss");
        motorcolect = hardwareMap.get(DcMotor.class, "motorcolect");
        motorext1 = hardwareMap.get(DcMotor.class, "motorext1");
        motordrop = hardwareMap.get(DcMotor.class,"motordrop");

        motords.setDirection(DcMotorSimple.Direction.REVERSE);
        motorss.setDirection(DcMotorSimple.Direction.REVERSE);

        motordf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motords.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorsf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorss.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorext1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motordrop.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motordf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motords.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorsf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorss.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorext1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motordrop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sysTimeC = System.currentTimeMillis();
        Chassis.start();
        cyclone.start();
    }

    @Override
    public void loop() {
        telemetry.addData("FPC: ",fpsC);
        telemetry.update();
    }
    public void POWER(double df ,double sf, double ds,double ss){
        motordf.setPower(df);
        motords.setPower(ds);
        motorsf.setPower(sf);
        motorss.setPower(ss);
    }
    public void stop(){
        stop = true;
    }
}
