package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
@Disabled
public class PID_4Bar extends OpMode {
    /**declare the motor*/
    private DcMotorEx motorBratStanga;

    boolean stop = false;
    /**variable for enabling the PID */
    boolean targetSet = false;
    /***/
    long target, position, encoderStanga, dt = 200;
    /** variable that holds the system current time in milliseconds*/
    double sysTime, targetTime;
    /**variables for calculating the pid*/
    double kp = 0, ki = 0, kd = 0;
    double p = 0 , i = 0 , d = 0 ;
    @Override
    public void init() {
        /**initialization motor  */
        motorBratStanga = hardwareMap.get(DcMotorEx.class,"bratStanga");
        /**setting the mode of the motor*/
        motorBratStanga.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        motorBratStanga.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        /**initialization the system current time milliseconds*/
        sysTime = System.currentTimeMillis();

        /**starting the threads*/
        brat.start();
        EncoderStanga.start();

    }
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
                    motorBratStanga.setPower(PID(es - target, kp, ki, kd));
                }
                /**calibrating the kp, ki , kd*/
                if (gamepad1.dpad_up && sysTime + dt < System.currentTimeMillis()) {
                    kp += 0.0001;
                    sysTime = System.currentTimeMillis();
                } else if (gamepad1.dpad_down && sysTime + dt < System.currentTimeMillis()) {
                    kp -= 0.0001;
                    sysTime = System.currentTimeMillis();
                }
                if (gamepad1.dpad_left && sysTime + dt < System.currentTimeMillis()) {
                    ki += 0.000001;
                    sysTime = System.currentTimeMillis();
                } else if (gamepad1.dpad_right && sysTime + dt < System.currentTimeMillis()) {
                    ki -= 0.000001;
                    sysTime = System.currentTimeMillis();
                }
                if (gamepad1.right_bumper && sysTime + 500 < System.currentTimeMillis()) {
                    kd += 0.0001;
                    sysTime = System.currentTimeMillis();
                } else if (gamepad1.left_bumper && sysTime + 500 < System.currentTimeMillis()) {
                    kd -= 0.0001;
                    sysTime = System.currentTimeMillis();
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

    /**using the stop function to stop the threads*/
    @Override
    public void stop(){
        stop = true;
    }


    /**using the loop function to send the telemetry to the phone*/
    @Override
    public void loop() {
        long es = 0, lastes;
        lastes = es;
        es = encoderStanga;
        telemetry.addData("kP", kp);
        telemetry.addData("kI", ki);
        telemetry.addData("kD", kd);
        telemetry.addData("target", target);
        telemetry.addData("position", es);
        telemetry.addData("deltaPos", lastes);
        telemetry.addData("power:", motorBratStanga.getPower());
        telemetry.update();
    }

    /**the pid function applies the pid algorithm to hold the arm in the same position*/
    private double PID (double delta, double kp, double ki, double kd){
        if(Math.abs(delta) > 5)i = 0;
        p = delta * kp; // se calibreaza prima prin kp
        i = (i + delta) * ki; //se calibreaza a doua prin ki, aduna eroarea in timp (cu cat e mai mult timp eroarea, cu atat corectia e mai mare)
        d = kd; // se calibreaza a 3 a, derivata lui f (x) = kd * x este kd
        return (p + i + d);
    }
}
