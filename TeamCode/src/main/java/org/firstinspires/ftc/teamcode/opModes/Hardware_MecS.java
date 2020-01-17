package org.firstinspires.ftc.teamcode.opModes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

/**
 * Created by Admin on 10/08/19
 */

public class Hardware_MecS extends LinearOpMode {

    private RevBulkData bulkData;
    private ExpansionHubMotor encoderDreapta, encoderSpate,encoderStanga;
    private ExpansionHubEx expansionHub;
    public DcMotorEx motorss;
    public DcMotorEx motorsf;
    public DcMotorEx motords;
    public DcMotorEx motordf;
    public boolean stop = false;
    public double t;
    long EncSp, EncSt, EncDr;
    public HardwareMap hwmap = null;
    public boolean startThread = false;
    private ElapsedTime period = new ElapsedTime();

    public Hardware_MecS(boolean startThreads) {
        this.startThread = startThreads;
    }
    public Hardware_MecS(){}

    @Override
    public void runOpMode() throws InterruptedException {

    }

    public void init(HardwareMap AHWmap) {
        hwmap = AHWmap;

        expansionHub = hwmap.get(ExpansionHubEx.class, "Expansion Hub Odometrie");

        /** NU CUMVA SA MAI SCRII hardwareMap in loc de hwmap*/
        /** si daca n-am chef? **/
        motordf = hwmap.get(DcMotorEx.class, "motordf");
        motords = hwmap.get(DcMotorEx.class, "motords");
        motorsf = hwmap.get(DcMotorEx.class, "motorsf");
        motorss = hwmap.get(DcMotorEx.class, "motorss");

        encoderDreapta = (ExpansionHubMotor) hardwareMap.get(DcMotor.class,"encoderDreapta");
        encoderSpate = (ExpansionHubMotor) hardwareMap.get(DcMotor.class,"encoderSpate");
        encoderStanga =(ExpansionHubMotor) hardwareMap.get(DcMotor.class,"encoderStanga");


        motordf.setPower(0);
        motords.setPower(0);
        motorsf.setPower(0);
        motorss.setPower(0);

        motords.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorss.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motordf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorsf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motords.setDirection(DcMotorSimple.Direction.REVERSE);
        motorss.setDirection(DcMotorSimple.Direction.REVERSE);

        motordf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motords.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorsf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorss.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motordf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motords.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorsf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorss.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(startThread) {
            Encodere.start();
        }

    }


    public void fata(double fata, double target) {
        //Rotatie inversa motorsf
        //        normala motordf
        //        inversa motorss
        //        normala motords
        // Invers Andymark
        stop_and_reset();
        target((int) (target * 25.868923), (int) (-target * 25.868923), (int) (- target * 25.868923), (int) (target * 25.868923));
        power(-fata, -fata, fata, fata);

        while (motordf.isBusy() && motords.isBusy() && motorsf.isBusy() && motorss.isBusy()) {
        }

        power(0, 0, 0, 0);

    }

    public void spate(double spate, int target) {
        stop_and_reset();
        target((int) (target * 25.868923), (int) (target * 25.868923), (int) (-target * 25.868923), (int) (-target * 25.868923));
        power(spate, spate, -spate, -spate);

        while (motordf.isBusy() && motords.isBusy() && motorsf.isBusy() && motorss.isBusy()) {
        }

        power(0, 0, 0, 0);
    }

    public void dreapta(double dreapta, int target) {
        stop_and_reset();
        target((int) (target * 25.868923), (int) (target * 25.868923), (int) (target * 25.868923), (int) (target * 25.868923));

        power(dreapta, dreapta, dreapta, dreapta);

        while (motordf.isBusy() && motords.isBusy() && motorsf.isBusy() && motorss.isBusy()) {
        }

        power(0, 0, 0, 0);
    }

    public void stanga(double stanga, int target) {
        stop_and_reset();

        target((int) (-target * 25.868923), (int) (-target * 25.868923), (int) (-target * 25.868923), (int) (-target * 25.868923));

        power(-stanga, -stanga, -stanga, -stanga);

        while (motordf.isBusy() && motords.isBusy() && motorsf.isBusy() && motorss.isBusy()) {
        }

        power(0, 0, 0, 0);
    }

    public void rotatie(double rotatie, double target) {
        stop_and_reset();
        target(-(int) (target * 9.25), (int) (target * 9.25), -(int) (target * 9.25), (int) (target * 9.25)); //9.035 pentru la fix

        power(rotatie * Math.signum(target), rotatie * Math.signum(target), rotatie * Math.signum(target), rotatie * Math.signum(target));

        while (motordf.isBusy() && motords.isBusy() && motorsf.isBusy() && motorss.isBusy()) {
        }
        power(0, 0, 0, 0);
        stop_and_reset();
    }
    //true pentru sens trigonometric, false pentru sensul acelor de ceasornic

    public void dd(double putere, int distanta, int sens) {
        stop_and_reset();

        motords.setTargetPosition((int) (distanta * sens * 25.868923));
        motorsf.setTargetPosition((int) (distanta * sens * 25.868923));

        power(0, 0, putere * sens, putere * sens);
        while (motords.isBusy() && motorsf.isBusy()) {
        }
        power(0, 0, 0, 0);
    }

    public void ds(double putere, int distanta, int sens) {
        stop_and_reset();

        motordf.setTargetPosition((int) (-distanta * sens * 25.868923));
        motorss.setTargetPosition((int) (distanta * sens * 25.868923));

        power(-putere * sens, 0, 0, putere * sens);
        while (motordf.isBusy() && motorss.isBusy()) {
        }
        power(0, 0, 0, 0);
    }

    public void waitForTick(long periodMs) {

        long remaining = periodMs - (long) period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }

    public void power(double df, double ss, double sf, double ds) {
        //++--
        motordf.setPower(df);
        motorss.setPower(ss);
        motorsf.setPower(sf);
        motords.setPower(ds);
    }

    public void stop_and_reset() {
        motordf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motords.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorsf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorss.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motordf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motords.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorsf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorss.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void target(int df, int ss, int sf, int ds) {
        motordf.setTargetPosition(df);
        motorss.setTargetPosition(ss);
        motorsf.setTargetPosition(sf);
        motords.setTargetPosition(ds);
    }

    public Thread Encodere = new Thread(new Runnable() {
        long encSpVal, encDrVal, encStVal;
        @Override
        public void run() {
            while (startThread) {
                bulkData = expansionHub.getBulkInputData();
                encSpVal = bulkData.getMotorVelocity(encoderSpate);
                encDrVal = bulkData.getMotorVelocity(encoderDreapta);
                encStVal = bulkData.getMotorVelocity(encoderStanga);
                EncSp = -encSpVal;
                EncSt = -encStVal;
                EncDr = encDrVal;
            }
        }
    });
}