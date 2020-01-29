package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.opencv.core.Mat;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import static java.lang.Math.PI;
import static java.lang.Math.abs;

@TeleOp
@Disabled
public class SinCosAxeBulk extends OpMode {
    private RevBulkData bulkData;
    /**declare the encoders*/
    private ExpansionHubMotor encoderDreapta, encoderSpate,encoderStanga;
    /**declare the expansionHub*/
    private ExpansionHubEx expansionHub;
    /**declare the motors */
    private ExpansionHubMotor motordf;
    private ExpansionHubMotor motorsf;
    private ExpansionHubMotor motords;
    private ExpansionHubMotor motorss;
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
    double x, y, encRot, lastAngle, correctedAngle;
    volatile double angle; // sens trigonometric +
    double d = 377, omniLengthMm = 188.49555921538759430775860299677, mmPerTick = omniLengthMm / 4000, rotationCircleLenght = PI * d, tickPerDeg = PIDControllerTestConfig.rotationCalib;
    long EncSp, EncSt, EncDr, lastEncDr;
    boolean stop = false;
    volatile boolean encodereCitite = false;
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
                forward = gamepad1.left_stick_y;
                right = -gamepad1.left_stick_x;
                clockwise = gamepad1.right_stick_x;

                /**calculating the power for motors */
                df = forward + clockwise - right;
                ss = -forward + clockwise + right;
                sf = -forward + clockwise - right;
                ds = forward + clockwise + right;

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


    private Thread Encoders = new Thread(new Runnable() {
        long encSpVal, encDrVal, encStVal;
        @Override
        public void run() {
            while (!stop) {
                if(!encodereCitite) {
                    bulkData = expansionHub.getBulkInputData();
                    encSpVal = bulkData.getMotorCurrentPosition(encoderSpate);
                    encDrVal = -bulkData.getMotorCurrentPosition(encoderDreapta);
                    encStVal = -bulkData.getMotorCurrentPosition(encoderStanga);
                    EncSp = encSpVal;
                    EncSt = encStVal;
                    EncDr = encDrVal;
                    encodereCitite = true;
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
                if(encodereCitite) {
                    /**pressing y resets the heading*/
                    if (gamepad1.y) {
                        offset = (EncDr - EncSt) / 2.0;
                        x=0;
                        y=0;
                    }
                    /**calculating the current heading in degrees*/
                    lastAngle = angle;

                    encRot = ((EncDr - EncSt) / 2.0) - offset;
                    angle = Math.toRadians(encRot / tickPerDeg) % (2 * PI);

                    lastCorrectedY = correctedY;
                    lastCorrectedX = correctedX;

                    correctedY = (EncSt + EncDr)/2.0;
                    correctedX = EncSp - PIDControllerTestConfig.sidewaysCalib * encRot;

                    deltaY = correctedY - lastCorrectedY;
                    deltaX = correctedX - lastCorrectedX;
                    correctedAngle = (lastAngle + angle)/2;

                    cosdeltay = (Math.cos(angle) * deltaY);
                    cosdeltax = (Math.cos((PI/2)- angle) * deltaX);
                    sindeltay = -(Math.sin(angle) * deltaY);
                    sindeltax = (Math.sin((PI/2)-angle) * deltaX);

                    y = y + (cosdeltay + cosdeltax) * mmPerTick;
                    x = x + (sindeltay + sindeltax) * mmPerTick;

                    encodereCitite = false;
                }
            }
        }
    });



    @Override
    public void init() {
        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub Odometrie");

        /**initialization motors */
        motordf = (ExpansionHubMotor) hardwareMap.get(DcMotor.class, "df");
        motords = (ExpansionHubMotor) hardwareMap.get(DcMotor.class, "ds");
        motorsf = (ExpansionHubMotor) hardwareMap.get(DcMotor.class, "sf");
        motorss = (ExpansionHubMotor) hardwareMap.get(DcMotor.class, "ss");

        /**initialization of the encoders*/
        encoderDreapta = motordf;
        encoderSpate = motorsf;
        encoderStanga = motorss;


        motords.setDirection(DcMotorSimple.Direction.REVERSE);
        motordf.setDirection(DcMotorSimple.Direction.REVERSE);//era ss

        /**set the mode of the  motors */


        motordf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motords.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorsf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorss.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motordf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motords.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorsf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorss.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /**setting the mode of the encoders*/

        expansionHub.setPhoneChargeEnabled(true);
        LOC.start();
        Encoders.start();
        Chassis.start();

    }
    /**using the loop function to send the telemetry to the phone*/
    @Override
    public void loop() {
        double X = x;
        double Y = y;
        telemetry.addData("X", X);
        telemetry.addData("Y", Y);
        telemetry.addData("angle", angle);
        telemetry.addData("EncDr: ",EncDr);
        telemetry.addData("EncSp: ",EncSp);
        telemetry.addData("EncSt: ",EncSt);
        telemetry.addData("MovingAngle:", Math.toDegrees(Math.atan(gamepad2.right_stick_x / (-gamepad2.right_stick_y))));
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
