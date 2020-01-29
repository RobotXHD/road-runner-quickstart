package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

@TeleOp
public class Pid_Brat extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    private double delta,enc,startTime,timeChange,lastTime,deltaSum,Power,lastDelta,dDelta;
    private boolean pid;
    private DcMotorEx motor;

    public void runOpMode(){
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PID.start();
        while(!isStarted()){
            telemetry.addData("enc", enc);
            telemetry.update();
        }
        waitForStart();
        pid = true;
        while(!isStopRequested()){}
    }
    public Thread PID = new Thread(new Runnable() {
        @Override
        public void run() {
            while(!isStopRequested()) {
                if(pid) {
                    delta = ConfigsTest.target - motor.getCurrentPosition();
                    startTime = System.nanoTime();
                    timeChange = (startTime - lastTime) / 1000.0;
                    deltaSum += (delta * timeChange);
                    dDelta = (delta - lastDelta) / timeChange;
                    lastDelta = delta;
                    lastTime = startTime;
                    Power = ConfigsTest.kp * delta + ConfigsTest.ki * deltaSum + ConfigsTest.kd * dDelta;
                    motor.setPower(Power);
                    TelemetryPacket packet = new TelemetryPacket();
                    packet.put("Delta Rotatie", delta);
                    packet.put("D Rotatie", ConfigsTest.kd * dDelta);
                    packet.put("I Rotatie", ConfigsTest.ki * deltaSum);
                    packet.put("P Rotatie", ConfigsTest.kp * delta);
                    packet.put("PID Rotatie", Power);
                    dashboard.sendTelemetryPacket(packet);
                }
            }
        }
    });
}

