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

@TeleOp
public class PIDControllerTest extends LinearOpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();
    public double encDr, encSt, encSp;
    public RevBulkData bulkData;
    public ExpansionHubMotor encoderDreapta, encoderSpate, encoderStanga;
    public ExpansionHubEx expansionHub;
    public ExpansionHubMotor motorss, motorsf, motords, motordf, motorScissorDr, motorScissorSt;
    public double rotatie = 0, ticksPerDegree = PIDControllerTestConfig.rotationCalib;
    public PIDControllerAdevarat pidRotatie = new PIDControllerAdevarat(0,0,0);
    public PIDControllerAdevarat pidY = new PIDControllerAdevarat(0,0,0);
    public PIDControllerAdevarat pidX = new PIDControllerAdevarat(0,0,0);
    public PIDControllerAdevarat pidScissor = new PIDControllerAdevarat(0,0,0);
    public double KPR = 0, KIR = 0, KDR = 0, SETPOINT = 0, correctionR;
    public double KPY = 0, KIY = 0, KDY = 0, SETPOINTY = 0, correctionY;
    public double KPX = 0, KIX = 0, KDX = 0, SETPOINTX = 0, correctionX;
    public double correctionScissor;
    public double ds, df, ss, sf, Y, tempRot, max;

    @Override
    public void runOpMode() throws InterruptedException {
        expansionHub = hardwareMap.get(ExpansionHubEx.class, configs.expansionHubOdometrieName);
        motorss = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, configs.ssName);
        motords = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, configs.dsName);
        motorsf = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, configs.sfName);
        motordf = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, configs.dfName);
        //dreapta =ds, stanga = ss, spate = sf;
        encoderDreapta = motordf;
        encoderSpate = motorss;
        encoderStanga = motorsf;

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

        pidRotatie.setSetpoint(0);
        pidY.setSetpoint(0);
        pidX.setSetpoint(0);

        pidRotatie.enable();
        pidY.enable();
        pidX.enable();
        while(!isStopRequested()){
            ticksPerDegree = PIDControllerTestConfig.rotationCalib;

            pidX.setTolerance(PIDControllerTestConfig.toleranceX);
            pidRotatie.setTolerance(PIDControllerTestConfig.toleranceRotatie);
            pidY.setTolerance(PIDControllerTestConfig.toleranceY);

            pidRotatie.setPID(PIDControllerTestConfig.p, PIDControllerTestConfig.i, PIDControllerTestConfig.d);
            pidRotatie.setSetpoint(PIDControllerTestConfig.setpoint);

            pidY.setPID(PIDControllerTestConfig.py, PIDControllerTestConfig.iy, PIDControllerTestConfig.dy);
            pidY.setSetpoint(PIDControllerTestConfig.setpointY);

            pidX.setPID(PIDControllerTestConfig.px, PIDControllerTestConfig.ix, PIDControllerTestConfig.dx);
            pidX.setSetpoint(PIDControllerTestConfig.setpointX);

            Y = (encDr + encSt)/2;
            correctionR = -pidRotatie.performPID(rotatie);
            correctionY = -pidY.performPID(Y);
            correctionX = pidX.performPID(encSp);

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

            power(ds, df, ss, sf);
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("P", pidRotatie.getP() * pidRotatie.getError());
            packet.put("I", pidRotatie.getI() * pidRotatie.getISum());
            packet.put("D", pidRotatie.getD() * pidRotatie.getDError());
            packet.put("PIDR", correctionR);
            packet.put("PIDY", correctionY);
            packet.put("PIDX", correctionX);
            packet.put("ErrorY", pidY.getError());
            packet.put("Y", Y);
            packet.put("encSp", encSp);
            packet.put("encDr", encDr);
            packet.put("encSt", encSt);
            packet.put("tempRot", rotatie);
            packet.put("rotationCalib", ticksPerDegree);
            packet.put("YonTarget",pidY.onTarget());
            packet.put("RotatieonTarget",pidRotatie.onTarget());
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
                dr = -bulkData.getMotorCurrentPosition(encoderDreapta);
                tempRot = ((dr - st)/2.0);
                rotatie = tempRot/ticksPerDegree;

                encDr = dr - tempRot;
                encSp = sp - rotatie * PIDControllerTestConfig.sidewaysCalib;
                encSt = st + tempRot;
            }
        }
    });
    private void power(double ds, double df, double ss, double sf) {
        motordf.setPower(df);
        motorss.setPower(ss);
        motorsf.setPower(sf);
        motords.setPower(ds);
    }
}
