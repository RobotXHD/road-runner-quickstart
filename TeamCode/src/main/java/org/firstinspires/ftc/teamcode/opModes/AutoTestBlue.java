package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class AutoTestBlue extends LinearOpMode {
    Hardware_Cam cam = new Hardware_Cam();
//    HardwareSkybot_V3 r = new HardwareSkybot_V3(true);
    private int caz;
    private double rotatie;
    @Override
    public void runOpMode() {
        cam.Init(hardwareMap);
       // r.Init(hardwareMap);
        cam.startDetection();
        sleep(1000);
    /*    while(cam.webcam.getFps() == 0){
            cam.stopDetection();
            cam.startDetection();
            sleep(1000);
        }*/
     /*   while (!isStarted()){
            telemetry.addData("X:", cam.skystoneDetectorModified.foundScreenPositions().get(0).x);
            telemetry.addData("Y:", cam.skystoneDetectorModified.foundScreenPositions().get(0).y);
           if(cam.skystoneDetectorModified.foundScreenPositions().get(0).y>143){
                telemetry.addData("Position", "LEFT");
                caz = 1;
            }
            else if(cam.skystoneDetectorModified.foundScreenPositions().get(0).y>32){
                telemetry.addData("Position", "CENTER");
                caz = 0;
            }
            else{
                telemetry.addData("Position", "RIGHT");
                caz = -1;

            }*/
            telemetry.update();
            /*telemetry.addData("DR", r.encDr);
            telemetry.addData("ST", r.encSt);
            telemetry.addData("SP", r.encSp);
            telemetry.update();
           /* telemetry.addData("Ceva: ", cam.skystoneDetectorModified.foundScreenPositions());
            telemetry.addData("Ceva v2: ", cam.skystoneDetectorModified.foundPozitionare());*/
        }
      //  waitForStart();
//        cam.stopDetection();

    /*    if(caz == -1){
            r.gotoY(3500,1);
            r.rotatie(-45,1);
            r.startColect();
            r.gotoY(20020,1);
            r.Colect(0.3);
            r.gotoY(-8000,1);
            r.rotatie(-45,1);
            r.gotoY(-53500,1);
            r.rotatie(-90,1);
            r.alinierePlaca(-5000,0.3);
            r.prindrePlate();
            r.startColectReverse();
            sleep(1500);
            r.Colect(-0.3);
            r.gotoX(-14000,1,250);
            r.gotoX(2000,1,250);
            rotatie = r.totalRot;
            r.rotatie(90,1,10);
            r.gotoY(-7000,1);
            r.desprindrePlate();
            sleep(1000);
            rotatie+=90;
            r.pidRotatie.setSetpoint(rotatie);
            r.stopColect();
            r.gotoX(5000,1);
            r.gotoY(35500, 1);
            r.gotoX(-12000,1);
            r.gotoX(3100,1);
            r.startColect();
            r.gotoY(5000,1);
            r.Colect(0.3);
            r.gotoX(8900, 1);
            r.rotatie(180,1);
            r.gotoY(25000,1);
            r.startColectReverse();
            r.gotoY(-10000,1);
            /*r.startColect();
            r.gotoY(   12000,1);
            r.Colect(0.3);
            r.gotoY(-12000,1);
            r.rotatie(-135,1);
            r.gotoX(-2000,1);
            r.gotoY(22500.0,1);
            r.startColectReverse();
            r.gotoY(-11000,1);

             */
       /* }

        else if(caz == 0){
            r.gotoY(8000, 1);
            r.rotatie(-45,1);
            r.startColect();
            r.gotoY(14010,1);
            r.Colect(0.3);
            r.gotoY(-8000,1);

            r.rotatie(-45,1);
            r.gotoY(-47250,1);
            r.rotatie(-90,1);
            r.alinierePlaca(-5000,0.3);
            r.prindrePlate();
            r.startColectReverse();
            sleep(1500);
            r.Colect(-0.3);
            r.gotoX(-14000,1,250);
            r.gotoX(2000,1,250);
            rotatie = r.totalRot;
            r.rotatie(90,1,10);
            r.gotoY(-7000,1);
            r.desprindrePlate();
            sleep(1000);
            rotatie+=90;
            r.pidRotatie.setSetpoint(rotatie);
            r.stopColect();
            r.gotoX(5000,1);
            r.gotoY(48000, 1);
            r.rotatie(-180,1);
            r.gotoX(6900,1);
            r.startColect();
            r.gotoY(5000,1);
            r.Colect(0.3);
            r.gotoX(-7900,1);
            r.gotoY(30000,1);
            r.startColectReverse();
            r.gotoY(-10000,1);
            /*
            r.startColect();
            r.gotoY(8500,1);
            r.Colect(0.3);
            r.gotoY(-8500,1);
            r.rotatie(-135,1);
            r.gotoX(-2000,1);
            r.gotoY(15000,1);
            r.startColectReverse();
            r.gotoY(-10000,1);
             */
      /*  }


        else{ //caz == "1"
            r.gotoY(12000, 1);
            r.rotatie(-45,1);
            r.startColect();
            r.gotoY(8000,1);
            r.Colect(0.3);
            r.gotoY(-8000,1);
            r.rotatie(-45,1);
            r.gotoY(-45000,1);
            r.rotatie(-90,1);
            r.alinierePlaca(-5000,0.3);
            r.prindrePlate();
            r.startColectReverse();
            sleep(1500);
            r.Colect(-0.3);
            r.gotoX(-14000,1,250);
            r.gotoX(2000,1,250);
            rotatie = r.totalRot;
            r.rotatie(90,1,5);
            r.gotoY(-7000,1);
            r.desprindrePlate();
            sleep(1000);
            rotatie+=90;
            r.pidRotatie.setSetpoint(rotatie);
            r.stopColect();
            r.gotoX(6500,1);
            r.gotoY(26000, 1);
            r.rotatie(45,1);
            r.startColect();
            r.gotoY(8500,1);
            r.Colect(0.3);
            r.gotoY(-8500,1);
            r.rotatie(135,1);
            r.gotoX(2000,1);
            r.gotoY(15000,1);
            r.startColectReverse();
            r.gotoY(-10000,1);
            // r.rotatie(-180,1);
        }-*/
    //}
}