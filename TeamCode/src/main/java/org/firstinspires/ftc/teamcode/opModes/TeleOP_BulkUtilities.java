package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;


public class TeleOP_BulkUtilities extends OpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();
    private RevBulkData bulkData;
    private ExpansionHubMotor encoderDreapta, encoderSpate, encoderStanga;
    private ExpansionHubEx expansionHubSasiu, expansionHubSisteme;
    private ExpansionHubMotor motorss, motorsf, motords, motordf, scissorSt, scissorDr, colectSt, colectDr;
    private volatile long encDr, encSt, encSp;
    private volatile boolean stop = false;

    private Thread encoderRead = new Thread(new Runnable() {
        @Override
        public void run() {
            while(!stop){
                bulkData = expansionHubSasiu.getBulkInputData();
                TelemetryPacket packet = new TelemetryPacket();
                packet.put("Motordf",bulkData.getMotorCurrentPosition(motordf));
                packet.put("Motorss",bulkData.getMotorCurrentPosition(motorss));
                packet.put("Motorsf",bulkData.getMotorCurrentPosition(motorsf));
                packet.put("Motords",bulkData.getMotorCurrentPosition(motords));
                bulkData = expansionHubSisteme.getBulkInputData();
                packet.put("scissorDr",bulkData.getMotorCurrentPosition(scissorDr));
                packet.put("scissorSt",bulkData.getMotorCurrentPosition(scissorSt));
                packet.put("colectDr",bulkData.getMotorCurrentPosition(colectDr));
                packet.put("colectSt",bulkData.getMotorCurrentPosition(colectSt));
                dashboard.sendTelemetryPacket(packet);
            }
        }
    });

    @Override
    public void init() {
        expansionHubSasiu = hardwareMap.get(ExpansionHubEx.class, configs.expansionHubOdometrieName);
        expansionHubSisteme = hardwareMap.get(ExpansionHubEx.class, configs.expansionHubSistemeName);
        motorss = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, configs.ssName);
        motords = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, configs.dsName);
        motorsf = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, configs.sfName);
        motordf = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, configs.dfName);

        scissorDr = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, configs.scissorDrName);
        scissorSt = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, configs.scissorStName);
        colectDr = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, configs.colectDrName);
        colectSt = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, configs.colectStName);

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

    }

    @Override
    public void stop() {
        stop = true;
    }
}
