package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;
@TeleOp
public class BulkTest extends OpMode {

    private RevBulkData bulkData;
    private ExpansionHubMotor encSp, encSt, encDr;
    private ExpansionHubEx expansionHubEx;
    private long encsp, encst, encdr;
    private boolean stop = false;
    private long fpsEnc, fpsEncLast, sysTimeEnc;
    @Override
    public void init() {
        expansionHubEx = hardwareMap.get(ExpansionHubEx.class, configs.expansionHubOdometrieName );
        encDr = (ExpansionHubMotor) hardwareMap.dcMotor.get(configs.encDrName);
        encSp = (ExpansionHubMotor) hardwareMap.dcMotor.get(configs.encSpName);
        encSt = (ExpansionHubMotor) hardwareMap.dcMotor.get(configs.encStName);

        sysTimeEnc = System.currentTimeMillis();

        expansionHubEx.setPhoneChargeEnabled(true);

        encodere.start();
    }

    private Thread encodere = new Thread(new Runnable() {
        long encSpVal, encDrVal, encStVal;
        @Override
        public void run() {
            while(!stop){
                bulkData = expansionHubEx.getBulkInputData();
                encSpVal = bulkData.getMotorCurrentPosition(encSp);
                encDrVal = bulkData.getMotorCurrentPosition(encDr);
                encStVal = bulkData.getMotorCurrentPosition(encSt);
                encsp = encSpVal;
                encst = encStVal;
                encdr = encDrVal;
                fpsEnc++;
                if(sysTimeEnc + 3000 < System.currentTimeMillis()){
                    fpsEncLast = fpsEnc/3;
                    fpsEnc = 0;
                    sysTimeEnc = System.currentTimeMillis();
                }
            }
        }
    });

    @Override
    public void stop(){
        stop = true;
    }

    @Override
    public void loop() {
        telemetry.addData("encSp:", encsp);
        telemetry.addData("encDr:", encdr);
        telemetry.addData("encSt:", encst);
        telemetry.addData("Hz:", fpsEncLast);
        telemetry.update();
    }
}
