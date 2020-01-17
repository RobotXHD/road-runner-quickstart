package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.cos;

@TeleOp
@Disabled
public class SinCosAxe extends OpMode {
    public DcMotor encoderDreapta, encoderSpate,encoderStanga;
    /**declare the motors*/
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
    private double forward, right, clockwise, tempforward, tempright;
    double lastCorrectedY = 0, lastCorrectedX = 0, correctedY = 0, correctedX = 0, deltaY, deltaX,sindeltay,sindeltax, cosdeltay,cosdeltax;
    double x, y, encRot;
    volatile double angle; // sens trigonometric +
    double d = 377, omniLengthMm = 188.49555921538759430775860299677, mmPerTick = omniLengthMm / 4000, rotationCircleLenght = PI * d, tickPerDeg = (rotationCircleLenght / omniLengthMm / 360) * 4000;
    long EncSp, EncSt, EncDr, lastEncDr;
    boolean stop = false;
    volatile boolean EncDrVerificare = false, EncStVerificare = false, EncSpVerificare = false;
    long timestamp, lastTimestamp,LAST;
    private Thread Chassis = new Thread( new Runnable() {
        @Override
        public void run() {
            /**repeat until the program stops*/
            while (!stop) {
                /**change the variable that controls the speed of the chassis using the bumpers*/
                if (gamepad1.right_bumper) {
                    v = 1;
                } else if (gamepad1.left_bumper) {
                    v = 2;
                }
                /**getting the gamepad joystick values*/
                tempforward = gamepad1.left_stick_y;
                tempright = -gamepad1.left_stick_x;
                clockwise = gamepad1.right_stick_x;

                forward = tempforward * Math.cos(angle) + tempright * Math.sin(angle);
                right = -tempforward * Math.sin(angle) + tempright * Math.cos(angle);


                /**calculating the power for motors */
                df = forward + clockwise - right;
                ss = forward - clockwise - right;
                sf = -forward + clockwise - right;
                ds = -forward - clockwise - right;

                /**normalising the power values*/
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
                /** setting the speed of the chassis*/
                if (v == 1) {
                    POWER(df / 5, sf / 5, ds / 5, ss / 5);
                } else if (v == 2) {
                    POWER(df, sf, ds, ss);
                }
            }
        }
    });

    private Thread ENCDR = new Thread(new Runnable() {
        long encdr;
        @Override
        public void run() {
            /**repeat until the program stops*/
            while(!stop){
                if(!EncDrVerificare) {
                    EncDrVerificare = true;
                    /**reading the encoders*/
                    encdr = encoderDreapta.getCurrentPosition();
                    /**assigning the encoders value to another global value*/
                    lastTimestamp = timestamp;
                    timestamp = System.currentTimeMillis();
                    LAST = timestamp-lastTimestamp;
                    lastEncDr = EncDr;
                    EncDr = encdr;
                }
            }
        }
    });

    private Thread ENCST = new Thread(new Runnable() {
        long encst;
        @Override
        public void run() {
            /**repeat until the program stops*/
            while(!stop){
                if(!EncStVerificare) {
                    EncStVerificare = true;
                    /**reading the encoders*/
                    encst = encoderStanga.getCurrentPosition();
                    /**assigning the encoders value to another global value*/
                    EncSt = -encst;
                }
            }
        }
    });
    private Thread ENCSP = new Thread(new Runnable() {
        long encsp;
        @Override
        public void run() {
            /**repeat until the program stops*/
            while(!stop){
                if(!EncSpVerificare) {
                    EncSpVerificare = true;
                    /**reading the encoders*/
                    encsp = encoderSpate.getCurrentPosition();
                    /**assigning the encoders value to another global value*/
                    EncSp = -encsp;
                }
            }
        }
    });

    private Thread LOC = new Thread(new Runnable() {
        double offset = 0;
        @Override
        public void run() {
            /**repeat until the program stop*/
            while (!stop) {
                if(EncStVerificare && EncDrVerificare && EncSpVerificare) {
                    /**pressing y resets the heading*/
                    if (gamepad1.y) {
                        offset = (EncDr - EncSt) / 2.0;
                        x=0;
                        y=0;
                    }
                    /**calculating the current heading in degrees*/
                    encRot = ((EncDr - EncSt) / 2.0) - offset;
                    angle = Math.toRadians(encRot / tickPerDeg) % (2 * PI);

                    lastCorrectedY = correctedY;
                    lastCorrectedX = correctedX;

                    correctedY = (EncSt + EncDr)/2.0;
                    correctedX = EncSp + 0.656 * encRot;

                    deltaY = correctedY - lastCorrectedY;
                    deltaX = correctedX - lastCorrectedX;

                    cosdeltay = (Math.cos(angle) * deltaY);
                    cosdeltax = (Math.cos((PI/2)- angle) * deltaX);
                    sindeltay = -(Math.sin(angle) * deltaY);
                    sindeltax = (Math.sin((PI/2)-angle) * deltaX);

                    y = y + (cosdeltay + cosdeltax) * mmPerTick;
                    x = x + (sindeltay + sindeltax) * mmPerTick ;

                    EncSpVerificare = false;
                    EncDrVerificare = false;
                    EncStVerificare = false;
                }
            }
        }
    });



    @Override
    public void init() {
        /**initialization motors */
        motordf = hardwareMap.get(DcMotorEx.class, "df");
        motords = hardwareMap.get(DcMotorEx.class, "ds");
        motorsf = hardwareMap.get(DcMotorEx.class, "sf");
        motorss = hardwareMap.get(DcMotorEx.class, "ss");

        /**initialization of the encoders*/
        encoderDreapta = hardwareMap.get(DcMotor.class,"encoderDreapta");
        encoderSpate = hardwareMap.get(DcMotor.class,"encoderSpate");
        encoderStanga = hardwareMap.get(DcMotor.class,"encoderStanga");


        motords.setDirection(DcMotorSimple.Direction.REVERSE);
        motorss.setDirection(DcMotorSimple.Direction.REVERSE);

        /**set the mode of the  motors */

        motordf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motords.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorsf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorss.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motordf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motords.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorsf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorss.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /**setting the mode of the encoders*/
        encoderDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        encoderSpate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoderDreapta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoderStanga.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /**starting the threads*/
        ENCDR.start();
        ENCSP.start();
        ENCST.start();
        LOC.start();
        Chassis.start();
    }
    /**using the loop function to send the telemetry to the phone*/
    @Override
    public void loop() {
      //  telemetry.addData("X", x);
        //telemetry.addData("Y", y);
        telemetry.addData("Degrees", (EncDr-lastEncDr) * 360/4000);
        telemetry.addData("EncDr: ",EncDr);
        telemetry.addData("EncSp: ",EncSp);
        telemetry.addData("EncSt: ",EncSt);
        telemetry.addData("Timestamp diff", LAST);
        telemetry.update();

    }

    /**the power function sets the motor's power*/
    public void stop(){
        stop = true;
    }

    /**the power function sets the motor's power*/
    public void POWER(double df1, double sf1, double ds1, double ss1){
        motordf.setPower(df1);
        motorss.setPower(ss1);
        motorsf.setPower(sf1);
        motords.setPower(ds1);
    }
}
