package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static java.lang.Math.PI;
import static java.lang.Math.abs;

@TeleOp
@Disabled
public class TeleOP_S4T_LOCC extends OpMode {
    /***declare the motors and encoders */
    private DcMotor encoderDreapta , encoderSpate, encoderStanga;
    private DcMotorEx motordf;
    private DcMotorEx motorsf;
    private DcMotorEx motords;
    private DcMotorEx motorss;
    /**variable for changing the movement speed of the robot*/
    private int v = 2;
    /**variables for calculating the power for motors*/
    private double df;
    private double sf;
    private double ds;
    private double ss;
    private double max;
    /**variables for holding the gamepad joystick values;
     * we don't want to access them too many times in a loop */
    private double forward, rright, clockwise;
    /**variable that stops the threads when programs stop*/
    private boolean stop = false;
    /**variables that count the thread's fps*/
    private long fpsC=0, fpsEncDr = 0, fpsEncSp = 0, fpsEncSt = 0, fpsLOC = 0, fpsLOCALIZARE = 0;
    private long fpsCLast, fpsEncSpLast, fpsEncDrLast, fpsEncStLast, fpsLOCLast, fpsLOCALIZARELast;
    /** variables that  hold the system current time milliseconds*/
    private long sysTimeC, sysTimeEncSp, sysTimeEncDr, sysTimeEncSt, sysTimeLOC, sysTimeLOCALIZARE,sysTimeLoop, sysTimeP, sysTimeI, sysTime, nextLoopTime = 3000, nextLoop;
    /**variables that hold the encoders value*/
    private int EncDr, EncSp, EncSt;
    /**variablesb for calculating the pid */
    private double angle, fs, sideways;
    private double d = 377, omniLenght = 188.49555921538759430775860299677, rotationCircleLenght = PI * d, tickPerDeg = (rotationCircleLenght / omniLenght / 360) * 4000;
    private double encRot, targetAngle, delta, p, i, kp = -0.16, ki = -0.3, kd, corr;
    /**variables used to synchronise the threads*/
    private boolean  EncDrVerificare = false, EncStVerificare = false, EncSpVerificare = false;


    private Thread ENCDr = new Thread(new Runnable() {
        int encdr;
        @Override
        public void run() {
            /**repeat until the program stops */
            while (!stop) {
                /**checking if the value was read by the rotation of the thread*/
                if (!EncDrVerificare) {
                    EncDrVerificare = true;
                    /**reading the encoder*/
                    encdr = encoderDreapta.getCurrentPosition();
                    /**assigning the encoders value to another global value*/
                    EncDr = encdr;
                }
                /**fps counter*/
                fpsEncDr++;
                if (sysTimeEncDr + 3000 < System.currentTimeMillis()) {
                    fpsEncDrLast = fpsEncDr / 3;
                    fpsEncDr = 0;
                    sysTimeEncDr = System.currentTimeMillis();
                }
            }
        }
    });

    private Thread ENCSp = new Thread(new Runnable() {
        int encsp;
        @Override
        public void run() {
            /**repeat until the program stops */
            while(!stop){
                /**checking if the value was read by the rotation of the thread*/
                if(!EncSpVerificare) {
                    EncSpVerificare = true;
                    /**reading the encoder*/
                    encsp = encoderSpate.getCurrentPosition();
                    /**assigning the encoders value to another global value*/
                    EncSp = encsp;
                }
                /**fps counter*/
                fpsEncSp++;
                if (sysTimeEncSp + 3000 < System.currentTimeMillis()) {
                        fpsEncSpLast = fpsEncSp / 3;
                        fpsEncSp = 0;
                        sysTimeEncSp = System.currentTimeMillis();
                }

            }

        }
    });

    private Thread ENCSt = new Thread(new Runnable() {
        int encst;
        @Override
        public void run() {
            /**repeat until the program stops */
            while(!stop) {
                /**checking if the value was read by the rotation of the thread*/
                if (!EncStVerificare) {
                    EncStVerificare = true;
                    /**reading the encoder*/
                    encst = encoderStanga.getCurrentPosition();
                    /**assigning the encoders value to another global value*/
                    EncSt = -encst;
                }
                /**fps counter*/
                fpsEncSt++;
                if (sysTimeEncSt + 3000 < System.currentTimeMillis()) {
                    fpsEncStLast = fpsEncSt / 3;
                    fpsEncSt = 0;
                    sysTimeEncSt = System.currentTimeMillis();
                }

            }
        }
    });


    private Thread Chassis = new Thread( new Runnable() {
        @Override
        public void run() {
            /**repeat until the program stops */
            while (!stop) {
                /**change the variable that controls the speed of the chassis using the bumpers*/
                if (gamepad1.right_bumper) {
                    v = 1;
                } else if (gamepad1.left_bumper) {
                    v = 2;
                }
                /**getting the gamepad joystick values*/
                forward = gamepad1.left_stick_y;
                rright = -gamepad1.left_stick_x;
                clockwise = gamepad1.right_stick_x;

                /**calculating the power for the motors*/
                df = forward + clockwise - rright;
                ss = forward - clockwise - rright;
                sf = -forward + clockwise - rright;
                ds = -forward - clockwise - rright;

                /**normalising the power values */
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

                /**setting the speed of the chassis*/
                if (v == 1) {
                    POWER(df / 5, sf / 5, ds / 5, ss / 5);
                } else if (v == 2) {
                    POWER(df, sf, ds, ss);
                }

                /**fps counter*/
                fpsC++;
                if (sysTimeC + 1000 < System.currentTimeMillis()) {
                    fpsCLast = fpsC;
                    fpsC = 0;
                    sysTimeC = System.currentTimeMillis();

                }
            }
        }
    });

    private Thread LOC = new Thread(new Runnable() {
        double offset = 0;
        @Override
        public void run() {
            /**repeat until the program stops */
            while (!stop) {
                /**pressing y resets the heading*/
                if(gamepad1.y){
                    offset = (EncDr - EncSt) / 2.0;
                }
                /**calculating the current heading in degrees*/
                encRot = ((EncDr - EncSt) / 2.0) - offset;
                angle = encRot / tickPerDeg;

                /**fps counter*/
                fpsLOC++;
                if (sysTimeLOC + 3000 < System.currentTimeMillis()) {
                    fpsLOCLast = fpsLOC/3;
                    fpsLOC = 0;
                    sysTimeLOC = System.currentTimeMillis();
                }
            }
        }
    });

    private Thread LOCALIZARE = new Thread(new Runnable() {
        @Override
        public void run() {
            /**repeat until the program stops */
            while(!stop){
                /***/
                Math.sin(angle);

                /**fps counter*/
                fpsLOCALIZARE++;
                if (sysTimeLOCALIZARE + 3000 < System.currentTimeMillis()) {
                    fpsLOCALIZARELast = fpsLOCALIZARE/3;
                    fpsLOCALIZARE = 0;
                    sysTimeLOCALIZARE = System.currentTimeMillis();
                }
            }

        }
    });


    @Override
    public void init() {

        /**initialization motors and the encoders */
        motordf = hardwareMap.get(DcMotorEx.class, "df");
        motords = hardwareMap.get(DcMotorEx.class, "ds");
        motorsf = hardwareMap.get(DcMotorEx.class, "sf");
        motorss = hardwareMap.get(DcMotorEx.class, "ss");

        encoderDreapta = hardwareMap.get(DcMotor.class,"encoderDreapta");
        encoderSpate = hardwareMap.get(DcMotor.class,"encoderSpate");
        encoderStanga = hardwareMap.get(DcMotor.class, "encoderStanga");

        motords.setDirection(DcMotorSimple.Direction.REVERSE);
        motorss.setDirection(DcMotorSimple.Direction.REVERSE);

        /**setting the mode of the motors and the mode of the encoders*/
        motordf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motords.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorsf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorss.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        encoderSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        encoderSpate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoderDreapta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoderStanga.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motordf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motords.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorsf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorss.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /**initialization the system current time milliseconds*/
        sysTime = System.currentTimeMillis();
        sysTimeC = sysTime;
        sysTimeEncDr = sysTime;
        sysTimeEncSp = sysTime;
        sysTimeEncSt = sysTime;
        sysTimeLOC = sysTime;
        sysTimeP = sysTime;
        sysTimeI = sysTime;
        sysTimeLOCALIZARE = sysTime;

        /**starting the threads*/
        ENCDr.start();
        ENCSp.start();
        ENCSt.start();
        Chassis.start();
        LOC.start();
        LOCALIZARE.start();
    }
    /**using the loop function to send the telemetry to the phone*/
    @Override
    public void loop() {
        telemetry.addData("ENCDr:", EncDr);
        telemetry.addData("ENCSp:", EncSp);
        telemetry.addData("ENCSt:", EncSt);
        telemetry.addData("EncRot", encRot);

        telemetry.addData("Angle:", angle);

        telemetry.addData("motordf: ", df);
        telemetry.addData("motorsf: ", sf);
        telemetry.addData("motords: ", ds);
        telemetry.addData("motorss: ", ss);

        telemetry.addData("p: ", p);
        telemetry.addData("i: ", i);
        telemetry.addData("kp: ", kp);
        telemetry.addData("ki: ", ki);

        /*telemetry.addData("Th Chassis: ", fpsCLast);
        telemetry.addData("Th EncSp: ", fpsEncSpLast);
        telemetry.addData("Th EncDr: ", fpsEncDrLast);
        telemetry.addData("Th EncSt: ", fpsEncStLast);
        telemetry.addData("Th LOC: ", fpsLOCLast);*/
        telemetry.addData("Th LOCALIZARE: ", fpsLOCALIZARELast);
        telemetry.update();
    }
    /**using the stop function to stop the threads*/
    public void stop(){stop = true;}

    /**the power function sets the motor's power*/
    public void POWER(double df1, double sf1, double ds1, double ss1){
        motordf.setPower(df1);
        motorss.setPower(ss1);
        motorsf.setPower(sf1);
        motords.setPower(ds1);
    }
}