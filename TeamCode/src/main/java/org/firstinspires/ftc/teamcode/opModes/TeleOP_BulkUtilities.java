package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

@TeleOp
public class TeleOP_BulkUtilities extends OpMode {

    private RevBulkData bulkData;
    private ExpansionHubMotor encoderDreapta, encoderSpate, encoderStanga;
    private ExpansionHubEx expansionHub;
    private ExpansionHubMotor motorss, motorsf, motords, motordf;
    private long encoderDrTotal,encoderStTotal, encoderSpTotal, rotatieTarget;
    private volatile double rotatie;
    public double ticksPerDegree = 70.2335277777777;//70.1445833333333333
    private volatile long encDr, encSt, encSp;
    private volatile boolean stop = false;

    private Thread encoderRead = new Thread(new Runnable() {
        long st, dr, sp;
        @Override
        public void run() {
            while(!stop){
                bulkData = expansionHub.getBulkInputData();
                st = bulkData.getMotorCurrentPosition(encoderStanga);
                sp = bulkData.getMotorCurrentPosition(encoderSpate);
                dr = bulkData.getMotorCurrentPosition(encoderDreapta);
                encDr = dr;
                encSp = sp;
                encSt = st;
                rotatie = ((dr - st)/2.0)/ticksPerDegree;
            }
        }
    });

    @Override
    public void init() {
        expansionHub = hardwareMap.get(ExpansionHubEx.class, configs.expansionHubOdometrieName);
        motorss = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, configs.ssName);
        motords = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, configs.dsName);
        motorsf = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, configs.sfName);
        motordf = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, configs.dfName);

        encoderDreapta = motorss;
        encoderSpate = motorsf;
        encoderStanga = motordf;

        motordf.setPower(0);
        motords.setPower(0);
        motorsf.setPower(0);
        motorss.setPower(0);

        motordf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motords.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorsf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorss.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motordf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motords.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorsf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorss.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motordf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motords.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorsf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorss.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motords.setDirection(DcMotorSimple.Direction.REVERSE);
        motordf.setDirection(DcMotorSimple.Direction.REVERSE);
        gamepad1.setJoystickDeadzone(0);

        encoderRead.start();
    }

    @Override
    public void loop() {
        motordf.setPower(-gamepad1.right_stick_x/3);
        motords.setPower(-gamepad1.right_stick_x/3);
        motorsf.setPower(gamepad1.right_stick_x/3);
        motorss.setPower(gamepad1.right_stick_x/3);
        telemetry.addData("EncoderDreapta", encDr);
        telemetry.addData("EncoderStanga", encSt);
        telemetry.addData("Rotatie", rotatie);
        telemetry.update();
    }

    @Override
    public void stop() {
        stop = true;
    }
}
