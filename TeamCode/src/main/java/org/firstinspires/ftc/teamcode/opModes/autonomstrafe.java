package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorEx;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
@Disabled
public class autonomstrafe extends LinearOpMode {

    private DcMotorEx motordf;
    private DcMotorEx motorsf;
    private DcMotorEx motords;
    private DcMotorEx motorss;
    private DcMotor motorColectSt, motorColectDr;
    long currentTime;
    @Override
    public void runOpMode() throws InterruptedException {
        motordf = hardwareMap.get(DcMotorEx.class, configs.dfName);
        motords = hardwareMap.get(DcMotorEx.class, configs.dsName);//encSt
        motorsf = hardwareMap.get(DcMotorEx.class, configs.sfName);//encDr
        motorss = hardwareMap.get(DcMotorEx.class, configs.ssName);//encSp

        motorColectDr = hardwareMap.get(DcMotor.class, configs.colectDrName);
        motorColectSt = hardwareMap.get(DcMotor.class, configs.colectStName);

        motords.setDirection(DcMotorSimple.Direction.REVERSE);
        motorss.setDirection(DcMotorSimple.Direction.REVERSE);

        motordf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motords.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorsf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorss.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motordf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motords.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorsf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorss.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while(!isStarted()){
            telemetry.addData("State", "Running");
            telemetry.update();
        }

        waitForStart();

        currentTime = System.currentTimeMillis();
        while(currentTime + 2000 > System.currentTimeMillis()){

            power(-0.5,-0.5, -0.5, -0.5);
        }
        currentTime = System.currentTimeMillis();
        while(currentTime + 1300 > System.currentTimeMillis()){

            power(-0.5,0.5, 0.5, -0.5);
        }
        power(0,0, 0, 0);
        motorColectDr.setPower(-1);
        sleep(1000);
        motorColectDr.setPower(0);
        sleep(1000);
    }
    private void power(double ds, double df, double ss, double sf){
        motordf.setPower(df);
        motorss.setPower(ss);
        motorsf.setPower(sf);
        motords.setPower(ds);
    }
}
