package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import static java.lang.Math.abs;

@TeleOp
@Disabled
public class TeleOP_ServoTOTAL extends OpMode {
    private RevBulkData bulkData;
    private ExpansionHubEx expansionHub;
    private ExpansionHubMotor motordf;
    private ExpansionHubMotor motorsf;
    private ExpansionHubMotor motords;
    private ExpansionHubMotor motorss;
    private ExpansionHubMotor motorColectSt, motorColectDr;
    private DcMotorEx motorBratStanga;
    boolean stop = false;
    boolean targetSet = false;
    long target, encoderStanga;
    double sysTime;
    double kp = 0, ki = 0, kd = 0;
    double p = 0 , i = 0 , d = 0 ;
    private int v = 2;
    private double df;
    private double sf;
    private double ds;
    private double ss;
    private double max;
    private double forward, rright, clockwise;
    private double sysTimeC,sysTimeP,systime;
    private boolean apoz = false, alast = true, apoz2 = false, alast2 = true;
    private double power = 1;
    public Servo servorot, servosj, servoclamp;
    public ServoImplEx servomonster;


    @Override
    public void init() {
        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub Odometrie");

        servorot = hardwareMap.get(Servo.class, "rot");
        servosj = hardwareMap.get(Servo.class, "sj");

        servomonster = hardwareMap.get(ServoImplEx.class, "monster");
        servomonster.setPwmRange(new PwmControl.PwmRange(750,2250));
        servoclamp = hardwareMap.get(Servo.class,"clamp");

        motordf =(ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, "df");
        motords = (ExpansionHubMotor)hardwareMap.get(DcMotorEx.class, "ds");
        motorsf = (ExpansionHubMotor)hardwareMap.get(DcMotorEx.class, "sf");
        motorss =(ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, "ss");

        motorColectDr = (ExpansionHubMotor)hardwareMap.get(DcMotor.class, "encoderDreapta");
        motorColectSt = (ExpansionHubMotor)hardwareMap.get(DcMotor.class, "encoderSpate");

        motorBratStanga = hardwareMap.get(DcMotorEx.class,"bratStanga");



        motords.setDirection(DcMotorSimple.Direction.REVERSE);
        motorss.setDirection(DcMotorSimple.Direction.REVERSE);

        motorBratStanga.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        motordf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBratStanga.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motords.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorsf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorss.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motordf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motords.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorsf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorss.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servomonster.setPosition(configs.pozitie_servoMonster_minim);
        servosj.setPosition(configs.pozitie_servoRot_maxim);
        //   servorot.setPosition(pozrotsus);
        systime = System.currentTimeMillis();
        sysTimeP = systime;
        sysTimeC = systime;
        sysTime = systime;
        Servo.start();
        brat.start();
        EncoderStanga.start();
        Chassis_Colect.start();
    }

    public Thread Servo = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
               /* if (systime + 200 < System.currentTimeMillis()) {
                    //monster
                    if (gamepad1.dpad_left && pozmonstersus < 1) {
                        pozmonstersus += 0.01;
                        servomonster.setPosition(pozmonstersus);
                        systime = System.currentTimeMillis();
                    } else if (gamepad1.dpad_right && pozmonstersus > 0.5) {
                        pozmonstersus -= 0.01;
                        servomonster.setPosition(pozmonstersus);
                        systime = System.currentTimeMillis();
                    }

                    if (gamepad1.dpad_up && pozmonsterjos < 0.5) {
                        pozmonsterjos += 0.01;
                        servomonster.setPosition(pozmonsterjos);
                        systime = System.currentTimeMillis();
                    } else if (gamepad1.dpad_down && pozmonsterjos > 0) {
                        pozmonsterjos -= 0.01;
                        servomonster.setPosition(pozmonsterjos);
                        systime = System.currentTimeMillis();
                    }

                    //sj
                    if (gamepad1.y && pozsjsus < 1) {
                        pozsjsus += 0.01;
                        servosj.setPosition(pozsjsus);
                        systime = System.currentTimeMillis();
                    } else if (gamepad1.a && pozsjsus > 0.5) {
                        pozsjsus -= 0.01;
                        servosj.setPosition(pozsjsus);
                        systime = System.currentTimeMillis();
                    }

                    if (gamepad1.b && pozsjjos < 0.5) {
                        pozsjjos += 0.01;
                        servosj.setPosition(pozsjjos);
                        systime = System.currentTimeMillis();
                    } else if (gamepad1.x && pozsjjos > 0) {
                        pozsjjos -= 0.01;
                        servosj.setPosition(pozsjjos);
                        systime = System.currentTimeMillis();
                    }
                }*/
                if (gamepad1.left_bumper) {
                    servosj.setPosition(configs.pozitie_servorSj_maxim);
                    servorot.setPosition(configs.pozitie_servoRot_maxim);
                } else if (gamepad1.right_bumper) {
                    servosj.setPosition(configs.pozitie_servorSj_minim);
                    servorot.setPosition(configs.pozitie_servoRot_minim);
                }

                if(gamepad1.dpad_up){
                    servomonster.setPosition(configs.pozitie_servoMonster_minim);
                }
                else if(gamepad1.dpad_down){
                    servomonster.setPosition(configs.pozitie_servoMonster_maxim);
                }

                if(gamepad1.a){
                    servoclamp.setPosition(1);
                }
                else if(gamepad1.b){
                    servoclamp.setPosition(0);
                }

            }
        }
    });

    private Thread Chassis_Colect = new Thread( new Runnable() {
        @Override
        public void run() {

            while (!stop) {
                if (gamepad1.dpad_left) {
                    v = 1;
                } else if (gamepad1.dpad_right) {
                    v = 2;
                }
                forward = gamepad1.left_stick_y;
                rright = -gamepad1.left_stick_x;
                clockwise = gamepad1.right_stick_x;

                df = forward + clockwise - rright;
                ss = forward - clockwise - rright;
                sf = -forward + clockwise - rright;
                ds = -forward - clockwise - rright;

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


                if (v == 1) {
                    POWER(df / 5, sf / 5, ds / 5, ss / 5);
                } else if (v == 2) {
                    POWER(df, sf, ds, ss);
                }

                boolean abut = gamepad1.x;
                if(alast != abut){
                    if(gamepad1.x) {
                        apoz = !apoz;
                        if (apoz){
                            motorColectSt.setPower(1);
                            motorColectDr.setPower(-1);
                        }
                        else{
                            motorColectSt.setPower(0);
                            motorColectDr.setPower(0);
                        }
                    }
                    alast=abut;
                }

                boolean abut2 = gamepad1.y;
                if(alast2 != abut2){
                    if(gamepad1.y) {
                        apoz2 = !apoz2;
                        if (apoz2){
                            motorColectSt.setPower(-1);
                            motorColectDr.setPower(1);
                        }
                        else{
                            motorColectSt.setPower(0);
                            motorColectDr.setPower(0);
                        }
                    }
                    alast2=abut2;
                }

            }
        }
    });

    private Thread brat = new Thread(new Runnable() {
        @Override
        public void run() {
            long es;
            /**repeat until the program stop*/
            while(!stop) {
                es = encoderStanga;
                /**pressing x or b on the gamepad lifts / lowers the arm, else we hold the position using PID*/
                if (gamepad1.right_trigger > 0) {
                    if (targetSet) {
                        targetSet = false;
                    }
                    motorBratStanga.setPower(-gamepad1.right_trigger);

                } else if (gamepad1.left_trigger > 0) {
                    if (targetSet) {
                        targetSet = false;
                    }
                    motorBratStanga.setPower(gamepad1.left_trigger);
                } else {
                    if (!targetSet) {
                        /*targetTime = System.currentTimeMillis();
                        motorBratStanga.setPower(0);
                        while(targetTime + 500 > System.currentTimeMillis()){}
                        */
                        target = es;
                        targetSet = true;
                    }
                    motorBratStanga.setPower(PID(es - target, -0.001, 0, 0));
                }
            }
        }
    });

    private Thread EncoderStanga = new Thread(new Runnable() {
        long es;
        @Override
        public void run() {
            /**repeat until the program stop*/
            while(!stop){
                /**reading the encoders*/
                es = motorBratStanga.getCurrentPosition();
                /**assigning the encoders value to another global value*/
                encoderStanga = es; // anti locking
            }
        }
    });

    @Override
    public void loop() {
        telemetry.addData("Rotpoz: ", servorot.getPosition());
        telemetry.addData("Rotsj: ", servosj.getPosition());
        telemetry.addData("Rotmonster: ", servomonster.getPosition());


        telemetry.addData("motordf: ", motordf.getCurrentPosition());
        telemetry.addData("motorsf: ", motorsf.getCurrentPosition());
        telemetry.addData("motords: ", motords.getCurrentPosition());
        telemetry.addData("motorss: ", motorss.getCurrentPosition());
        telemetry.addData("MSS current", motorss.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
        telemetry.addData("MSF current", motorsf.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
        telemetry.addData("MDS current", motords.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
        telemetry.addData("MDF current", motordf.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
        telemetry.addData("MCS current", motorColectSt.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
        telemetry.addData("MCD current", motorColectDr.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
        telemetry.addData("POWER: ", power);
        telemetry.update();
    }

    public void POWER(double df1, double sf1, double ds1, double ss1){
        motordf.setPower(df1);
        motorss.setPower(ss1);
        motorsf.setPower(sf1);
        motords.setPower(ds1);
    }

    private double PID (double delta, double kp, double ki, double kd){
        p = delta * kp; // se calibreaza prima prin kp
        i = (i + delta) * ki; //se calibreaza a doua prin ki, aduna eroarea in timp (cu cat e mai mult timp eroarea, cu atat corectia e mai mare)
        d = kd; // se calibreaza a 3 a, derivata lui f (x) = kd * x este kd
        return (p + i + d);
    }

    @Override
    public void stop() {
        stop = true;
    }
}
