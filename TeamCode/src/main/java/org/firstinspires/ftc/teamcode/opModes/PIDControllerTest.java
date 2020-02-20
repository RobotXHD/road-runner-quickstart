package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;


public class PIDControllerTest extends LinearOpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();
    public double encDr, encSt, encSp;
    public RevBulkData bulkData;
    public ExpansionHubMotor encoderDreapta, encoderSpate, encoderStanga;
    public ExpansionHubEx expansionHub;
    public ExpansionHubMotor motorss, motorsf, motords, motordf, motorScissorDr, motorScissorSt;
    public double KPR = 0, KIR = 0, KDR = 0, SETPOINT = 0, correctionR;
    public double KPY = 0, KIY = 0, KDY = 0, SETPOINTY = 0, correctionY;
    public double KPX = 0, KIX = 0, KDX = 0, SETPOINTX = 0, correctionX;
    public double correctionScissor;
    public double ds, df, ss, sf, Y, tempRot, max;

    @Override
    public void runOpMode() throws InterruptedException {
        expansionHub = hardwareMap.get(ExpansionHubEx.class, configs.expansionHubSistemeName);
        motorss = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, configs.ssName);
        motords = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, configs.dsName);
        motorsf = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, configs.sfName);
        motordf = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, configs.dfName);
        //dreapta =ds, stanga = ss, spate = sf;
        encoderDreapta =(ExpansionHubMotor) hardwareMap.get(DcMotorEx.class,configs.encDrName);
        encoderStanga =(ExpansionHubMotor) hardwareMap.get(DcMotorEx.class,configs.encStName);
        encoderSpate =(ExpansionHubMotor) hardwareMap.get(DcMotorEx.class,configs.encSpName);


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

        encoderRead.start();
        waitForStart();

        while(!isStopRequested()){
            ds = correctionR + correctionY - correctionX;
            df = correctionR + correctionY + correctionX;
            ss = -correctionR + correctionY + correctionX;
            sf = -correctionR + correctionY - correctionX;

            max = Math.abs(ds);
            max = Math.abs(df) > max ? Math.abs(df):max;
            max = Math.abs(sf) > max ? Math.abs(sf):max;
            max = Math.abs(ss) > max ? Math.abs(ss):max;
            if(max > 1){
                ds /= max;
                df /= max;
                sf /= max;
                ss /= max;
            }

            // power(ds, df, ss, sf);
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("PIDR", correctionR);
            packet.put("PIDY", correctionY);
            packet.put("PIDX", correctionX);
            packet.put("Y", Y);
            packet.put("encSp", encSp);
            packet.put("encDr", encDr);
            packet.put("encSt", encSt);
            dashboard.sendTelemetryPacket(packet);
        }
    }

    private Thread encoderRead = new Thread(new Runnable() {
        long st, dr, sp;
        @Override
        public void run() {
            while(!isStopRequested()){
                bulkData = expansionHub.getBulkInputData();
                st = -bulkData.getMotorCurrentPosition(encoderStanga);
                sp = bulkData.getMotorCurrentPosition(encoderSpate);
                dr = bulkData.getMotorCurrentPosition(encoderDreapta);

                encDr = dr;
                encSp = sp;
                encSt = st;
            }
        }
    });
}
