package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.openftc.revextensions2.ExpansionHubEx;

import static java.lang.Math.abs;

@TeleOp
public class TeleOp_Colect extends OpMode {
    /**
     * declare the motors and expansionHub and the servos
     */
    private ExpansionHubEx expansionHubOdometrie, expansionHubSisteme;
    private DcMotorEx motordf;
    private DcMotorEx motorsf;
    private DcMotorEx motords;
    private DcMotorEx motorss;
    private DcMotorEx scissorDreapta;
    private DcMotorEx scissorStanga;
    private DcMotor motorColectSt, motorColectDr;
    private Servo servoPlatformaSt, servoPlatformaDr, servoCapstone, servoParcareBlue, servoParcareRed;
    private ServoImplEx vexSt, vexDr, servoClamp;
    /**
     * variable for changing the movement speed of the robot
     */
    private int v = 2;
    /**
     * variables for calculating the power for motors
     */
    private double df;
    private double sf;
    private double ds;
    private double ss;
    private double max;
    private double sysTime;
    // private long encScissorDr, encScissorSt, offsetDr = 0, offsetSt = 0;
    /**
     * variables for holding the gamepad joystick values;
     * we don't want to access them too many times in a loop
     */
    private double forward, right, clockwise, powerLimit = 1, leftStickY, servoParcarePosition = 0.4;
    private boolean stop;
    private boolean apoz = false, alast = true, apoz2 = false, alast2 = true, apoz3 = false, alast3 = true, eStrans = false;
    private double powerColect = 1, powerSlider, scissorStangaOffset = 0;
    private TouchSensor touchScissorDr, touchScissorSt, touchGheara;
    private volatile double expansionHubOdometrieCurrent, expansionHubSistemeCurrent;
    private AnalogInput pot;

    private Thread Colect = new Thread(() -> {
        /**repeat until the program stops*/
        while (!stop) {
            /**reseting the encoders for the scissor motor*/
            if (touchScissorDr.isPressed() || touchScissorSt.isPressed()) {
                scissorStangaOffset = scissorStanga.getCurrentPosition();
            }

            leftStickY = -gamepad2.left_stick_y;
            scissorStanga.setPower(leftStickY);
            scissorDreapta.setPower(leftStickY);


            /**setting the collector motors on or off using the toggle*/
            boolean abut = gamepad2.b;
            if (alast != abut) {
                if (gamepad2.b) {
                    apoz = !apoz;
                    if (apoz && !touchGheara.isPressed()) {
                        motorColectSt.setPower(-powerColect);
                        motorColectDr.setPower(powerColect);
                        servoClamp.setPosition(configs.pozitie_servoClamp_desprindere);
                        apoz3 = false;
                    } else {
                        motorColectSt.setPower(0);
                        motorColectDr.setPower(0);
                    }
                }
                alast = abut;
            }

            boolean abut2 = gamepad2.x;
            if (alast2 != abut2) {
                if (gamepad2.x) {
                    apoz2 = !apoz2;
                    if (apoz2 && !touchGheara.isPressed()) {
                        motorColectSt.setPower(powerColect);
                        motorColectDr.setPower(-powerColect);
                        //         servoClamp.setPosition(0.65);
                    } else {
                        motorColectSt.setPower(0);
                        motorColectDr.setPower(0);
                    }
                }
                alast2 = abut2;
            }

            boolean abut3 = gamepad2.y;
            if (alast3 != abut3) {
                if (gamepad2.y) {
                    apoz3 = !apoz3;
                    if (apoz3) {
                        servoClamp.setPosition(configs.pozitie_servoClamp_prindere);
                    } else {
                        servoClamp.setPosition(configs.pozitie_servoClamp_desprindere);
                    }
                }
                alast3 = abut3;
            }

            /**closing the clamp automatically and stopping the cube collecting motors when the claw's switch is pressed by the cube */
            if (touchGheara.isPressed() && motorColectDr.getPower() != 0) {
                servoClamp.setPosition(configs.pozitie_servoClamp_prindere);
                motorColectSt.setPower(0);
                motorColectDr.setPower(0);
                apoz3 = true;
            }


            powerSlider = gamepad2.right_stick_y;
            if (powerSlider != 0) {
                vexDr.setPosition(0.5 + powerSlider / 2);
                vexSt.setPosition(0.5 - powerSlider / 2);
            } else {
                vexDr.setPosition(0.5);
                vexSt.setPosition(0.5);
            }

            if (gamepad1.dpad_down) {
                servoPlatformaDr.setPosition(configs.pozitie_servoPlatformaDr_prindere);
                servoPlatformaSt.setPosition(configs.pozitie_servoPlatformaSt_prindere);
            } else if (gamepad1.dpad_up) {
                servoPlatformaDr.setPosition(configs.pozitie_servoPlatformaDr_pliere);
                servoPlatformaSt.setPosition(configs.pozitie_servoPlatformaSt_pliere);
            }

            if (gamepad2.left_trigger > 0.9 && servoParcarePosition < 1) {
                 servoParcareBlue.setPosition(configs.pozitie_servoParcareBlue_desprindere);
            }

            if(gamepad2.right_trigger > 0.9){
                servoCapstone.setPosition(0);
            }
            else{
                servoCapstone.setPosition(configs.pozitie_servoCapstone_prindere
                );
            }
        }

    });

    private Thread Chassis = new Thread(() -> {
        /**repeat until the program stops*/
        while (!stop) {
            if (gamepad1.right_bumper) {
                v = 1;
            } else if (gamepad1.left_bumper) {
                v = 2;
            }
            /**getting the gamepad joystick values*/
            forward = -gamepad1.left_stick_y;
            right = gamepad1.left_stick_x;
            clockwise = -gamepad1.right_stick_x;
            /**limiting the power for the chassis expansionhub*/
            if (expansionHubSistemeCurrent > 5) {
                powerLimit = expansionHubOdometrieCurrent > 2 ? 2 / expansionHubOdometrieCurrent : 1;
            } else {
                powerLimit = 1;
            }

            /**calculating the power for motors */

            df = (forward + clockwise - right) * powerLimit;
            ss = (forward - clockwise - right) * powerLimit;
            sf = (-forward + clockwise - right) * powerLimit;
            ds = (-forward - clockwise - right) * powerLimit;

            /**normalising the power values*/
            max = abs(sf);
            if (abs(ds) > max) {
                max = abs(ds);
            }
            if (abs(df) > max) {
                max = abs(df);
            }
            if (abs(ss) > max) {
                max = abs(ss);
            }
            if (max > 1) {
                sf /= max;
                df /= max;
                ss /= max;
                ds /= max;
            }
            /** setting the speed of the chassis*/
            if (v == 1) {
                POWER(df / 2.2, sf / 2.2, ds / 2.2, ss / 2.2);
            } else if (v == 2) {
                POWER(df, sf, ds, ss);
            }
        }
    });
    /**
     * measure the current draw for each of expansionHub
     */
    private Thread current = new Thread(() -> {
        while (!stop) {
            expansionHubSistemeCurrent = expansionHubSisteme.getTotalModuleCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS);
            expansionHubOdometrieCurrent = expansionHubOdometrie.getTotalModuleCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS);
        }
    });

    /* public Thread automation = new Thread(new Runnable() {
         @Override
         public void run() {
             while (!stop) {
                 if (touchGheara.isPressed()) {
                     apoz3 = false;
                     servoClamp.setPosition(1);
                     motorColectDr.setPower(0);
                     motorColectSt.setPower(0);
                 }
             }
         }
     });*/
    @Override
    public void init() {
        /**initialization motors*/

        expansionHubOdometrie = hardwareMap.get(ExpansionHubEx.class, configs.expansionHubOdometrieName);
        expansionHubSisteme = hardwareMap.get(ExpansionHubEx.class, configs.expansionHubSistemeName);

        motordf = hardwareMap.get(DcMotorEx.class, configs.dfName);
        motords = hardwareMap.get(DcMotorEx.class, configs.dsName);//encSt
        motorsf = hardwareMap.get(DcMotorEx.class, configs.sfName);//encDr
        motorss = hardwareMap.get(DcMotorEx.class, configs.ssName);//encSp

        motorColectDr = hardwareMap.get(DcMotor.class, configs.colectDrName);
        motorColectSt = hardwareMap.get(DcMotor.class, configs.colectStName);

        scissorDreapta = hardwareMap.get(DcMotorEx.class, configs.scissorDrName);
        scissorStanga = hardwareMap.get(DcMotorEx.class, configs.scissorStName);

        servoClamp = hardwareMap.get(ServoImplEx.class, configs.servoClampName);
        servoPlatformaDr = hardwareMap.servo.get(configs.servoPlatformaDrName);
        servoPlatformaSt = hardwareMap.servo.get(configs.servoPlatformaStName);
        servoCapstone = hardwareMap.servo.get(configs.servoCapstoneName);
        pot = hardwareMap.analogInput.get(configs.potentiometruName);
        servoParcareBlue = hardwareMap.servo.get(configs.servoParcareBlueName);
        servoParcareRed = hardwareMap.servo.get(configs.servoParcareRedName);

        vexDr = hardwareMap.get(ServoImplEx.class, "vexDr");
        vexSt = hardwareMap.get(ServoImplEx.class, "vexSt");

        touchGheara = hardwareMap.touchSensor.get(configs.touchGhearaName);
        touchScissorDr = hardwareMap.touchSensor.get(configs.touchScissorDrName);
        touchScissorSt = hardwareMap.touchSensor.get(configs.touchScissorStName);

        motords.setDirection(DcMotorSimple.Direction.REVERSE);
        motorss.setDirection(DcMotorSimple.Direction.REVERSE);
        scissorDreapta.setDirection(DcMotorSimple.Direction.REVERSE);
        scissorStanga.setDirection(DcMotorSimple.Direction.REVERSE);

        servoClamp.setPwmRange(new PwmControl.PwmRange(750, 2250));
        vexDr.setPwmRange(new PwmControl.PwmRange(1000, 2000));
        vexSt.setPwmRange(new PwmControl.PwmRange(1000, 2000));

        /**setting the mode of the motors*/
        scissorStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        scissorStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        scissorDreapta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        scissorStanga.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motordf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motords.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorsf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorss.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        scissorDreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        scissorStanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motordf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motords.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorsf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorss.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sysTime = System.currentTimeMillis();
        servoClamp.setPosition(configs.pozitie_servoClamp_desprindere);
        servoCapstone.setPosition(configs.pozitie_servoCapstone_prindere);
        //  servoParcare.setPosition(servoParcarePosition);
        /**start the thread*/
        Colect.start();
        Chassis.start();
        current.start();

        telemetry.setMsTransmissionInterval(25);
        //automation.start();
    }

    /**
     * using the loop function to send the telemetry to the phone
     */
    @Override
    public void loop() {
        telemetry.addData("TouchscDr", touchScissorDr.isPressed());
        telemetry.addData("TouchscSt", touchScissorSt.isPressed());
        telemetry.addData("TouchGh", touchGheara.isPressed());
        telemetry.addData("scSt", scissorStanga.getCurrentPosition() - scissorStangaOffset);
        telemetry.addData("power df: ", motordf.getPower());
        telemetry.addData("potentiometru", pot.getVoltage());
        telemetry.update();
    }

    /**
     * using the stop function to stop the thread
     */
    public void stop() {
        stop = true;
    }

    /**
     * the power function sets the motor's power
     */
    public void POWER(double df1, double sf1, double ds1, double ss1) {
        motordf.setPower(df1);
        motorss.setPower(ss1);
        motorsf.setPower(sf1);
        motords.setPower(ds1);
    }

}
