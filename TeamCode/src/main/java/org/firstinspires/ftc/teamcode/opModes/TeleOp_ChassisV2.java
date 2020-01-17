package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
@Disabled
public class TeleOp_ChassisV2 extends OpMode {
    /**declare the motors*/
    private DcMotor motordf;
    private DcMotor motorsf;
    private DcMotor motords;
    private DcMotor motorss;
    /**variable that stops the threads when programs stop*/
    private boolean stop = false;
    /**variables for holding the gamepad joystick values;
     * we don't want to access them too many times in a loop */
    private double forward, sideways, rotation;
    /**variables for calculating the power for motors*/
    private double ds, df, ss, sf, max;
    /**variables that count the thread's fps*/
    private long fpsC=0, fps = 0;
    private long fpsCLast, fpsLast;
    /** variable that  holds the system current time milliseconds*/
    private long systimeC, systime;

    public Thread chassis = new Thread(new Runnable() {
        double dss, dff, sss, sff;
        @Override
        public void run() {
            /**repeat until the program stops*/
            while(!stop){
                /**getting the gamepad joystick values*/
                forward = -gamepad1.left_stick_y;
                sideways = gamepad1.left_stick_x;
                rotation = gamepad1.right_stick_x;

                /**calculating the power for motors */
                dss = forward + sideways - rotation;
                dff = forward - sideways - rotation;
                sss = forward - sideways + rotation;
                sff = forward + sideways + rotation;

                /**normalising the power values*/
                max = Math.abs(dss);
                max = max < Math.abs(dff) ? Math.abs(dff) : max;
                max = max < Math.abs(sss) ? Math.abs(sss) : max;
                max = max < Math.abs(sff) ? Math.abs(sff) : max;

                if(max > 1){
                    dss /= max;
                    dff /= max;
                    sss /= max;
                    sff /= max;
                }

                /**assigning the power value to another global value*/
                ds = dss;
                df = dff;
                ss = sss;
                sf = sff;

                /**fps counter*/
                fpsC++;
                if (systimeC + 1000 < System.currentTimeMillis()) {
                    fpsCLast = fpsC;
                    fpsC = 0;
                    systimeC = System.currentTimeMillis();
                }
            }
        }
    });
    public Thread Motordf = new Thread(new Runnable() {
        double power;
        @Override
        public void run() {
            /**repeat until the program stops*/
            while(!stop){
                /**setting the motor's power*/
                power = df;
                motordf.setPower(power);

                /**fps counter*/
                fps++;
                if (systime + 1000 < System.currentTimeMillis()) {
                    fpsLast = fps;
                    fps = 0;
                    systime = System.currentTimeMillis();
                }
            }
        }
    });
    public Thread Motords = new Thread(new Runnable() {
        double power;
        @Override
        public void run() {
            /**repeat until the program stops*/
            while(!stop){
                /**setting the motor's power*/
                power = ds;
                motords.setPower(power);
            }
        }
    });
    public Thread Motorsf = new Thread(new Runnable() {
        double power;
        @Override
        public void run() {
            /**repeat until the program stops*/
            while(!stop){
                /**setting the motor's power*/
                power = sf;
                motorsf.setPower(power);
            }
        }
    });

    public Thread Motorss = new Thread(new Runnable() {
        double power;
        @Override
        public void run() {
            /**repeat until the program stops*/
            while(!stop){
                /**setting the motor's power*/
                power = ss;
                motorss.setPower(power);
            }
        }
    });


    @Override
    public void init() {
        /**initialization motors  */
        motordf = hardwareMap.get(DcMotor.class, "df");
        motords = hardwareMap.get(DcMotor.class, "ds");
        motorsf = hardwareMap.get(DcMotor.class, "sf");
        motorss = hardwareMap.get(DcMotor.class, "ss");

        motorsf.setDirection(DcMotorSimple.Direction.REVERSE);
        motorss.setDirection(DcMotorSimple.Direction.REVERSE);

        /**set the mode of the motors */
        motordf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motords.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorsf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorss.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motordf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motords.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorsf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorss.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /**start the threads*/
        chassis.start();
        Motordf.start();
        Motords.start();
        Motorsf.start();
        Motorss.start();

        /**initialization system current milliseconds */
        systime = System.currentTimeMillis();
        systimeC = systime;
    }

    /**using the loop function to send the telemetry to the phone*/
    @Override
    public void loop() {
        telemetry.addData("fpsDf: ",fpsLast);
        telemetry.addData("fpsCH: ",fpsCLast);
        telemetry.update();
    }

    /**using the stop function to stop the threads*/
    @Override
    public void stop() {
        stop = true;
    }
}
