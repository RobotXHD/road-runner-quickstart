package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.openftc.revextensions2.VexMC29;

@TeleOp
@Disabled
public class scissorTest extends OpMode {
  //  private ServoImplEx vexSt, vexDr;
    private DcMotor scissorDreapta, scissorStanga;
    private DcMotor  motorColectSt, motorColectDr;
  //  private Servo servoclamp;
      private double systime;
    private boolean apoz = false, alast = true, apoz2 = false, alast2 = true;

    @Override
    public void init() {
        //scissorDreapta = hardwareMap.dcMotor.get("scissorDreapta");
        scissorStanga = hardwareMap.dcMotor.get("scissorStanga");
        motorColectDr = hardwareMap.get(DcMotor.class, "encoderDreapta");
        motorColectSt = hardwareMap.get(DcMotor.class, "encoderSpate");
        /*servoclamp = hardwareMap.servo.get("clamp");
        vexDr = hardwareMap.get(ServoImplEx.class, "vexDr");
        vexSt = hardwareMap h,.get(ServoImplEx.class, "vexSt");*/

     /*   vexDr.setPwmRange(new PwmControl.PwmRange(1000,2000));
        vexSt.setPwmRange(new PwmControl.PwmRange(1000,2000));*/

       // scissorDreapta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        scissorStanga.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

      //  scissorDreapta.setDirection(DcMotorSimple.Direction.REVERSE);
        scissorStanga.setDirection(DcMotorSimple.Direction.REVERSE);

        //scissorDreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        scissorStanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

       // systime = System.currentTimeMillis();
    }

    @Override
    public void loop() {

        scissorStanga.setPower(gamepad2.right_stick_y);
      //  scissorDreapta.setPower(gamepad2.right_stick_y);
    /*    if(gamepad2.a){
            servoclamp.setPosition(configs.pozitie_servoClamp_minim);
        }
        else if(gamepad2.b){
            servoclamp.setPosition(configs.pozitie_servoClamp_maxim);
        }
        if(gamepad2.dpad_up){
            vexDr.setPosition(1);
            vexSt.setPosition(0);
        }
        else if(gamepad2.dpad_down){
            vexDr.setPosition(1);
            vexSt.setPosition(0);
        }
        else {
            vexSt.setPosition(0.5);
            vexDr.setPosition(0.5);
        }
        */

        boolean abut = gamepad2.x;
        if(alast != abut){
            if(gamepad2.x) {
                apoz = !apoz;
                if (apoz){
                    motorColectSt.setPower(0.6);
                    motorColectDr.setPower(-0.6);
                }
                else{
                    motorColectSt.setPower(0);
                    motorColectDr.setPower(0);
                }
            }
            alast=abut;
        }

        boolean abut2 = gamepad2.y;
        if(alast2 != abut2){
            if(gamepad2.y) {
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
     //   telemetry.addData("DreaptaPoz:", scissorDreapta.getCurrentPosition());
        telemetry.addData("StangaPoz:", scissorStanga.getCurrentPosition());
       // telemetry.addData("DreaptaPower:", scissorDreapta.getPower());
        telemetry.addData("StangaPower:", scissorStanga.getPower());
      //  telemetry.addData("clamp: ",servoclamp.getPosition());
        telemetry.update();
    }
}
