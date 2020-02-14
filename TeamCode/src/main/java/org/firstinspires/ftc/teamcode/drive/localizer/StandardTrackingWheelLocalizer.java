package org.firstinspires.ftc.teamcode.drive.localizer;

import android.support.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.opModes.configs;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.Arrays;
import java.util.List;

/*
 * Tracking wheel localizer implementation assuming the following configuration:
 *
 *    /--------------\
 *    |              |
 *    |              |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |     ====     |
 *    \--------------/
 *
 * Note: this could be optimized significantly with REV bulk reads
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 4000;
    public static double WHEEL_RADIUS = 3; // cm
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 40.87; // cm; distance between the left and right wheels 405/2.54
    public static double BACKWARDS_OFFSET = 12.5; // cm; offset of the lateral wheel

    public ExpansionHubEx expansionHub;
    public RevBulkData bulkData;

    private ExpansionHubMotor leftEncoder, rightEncoder, backEncoder;


    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap, DcMotorEx encLeft, DcMotorEx encRight, DcMotorEx encBack) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(-BACKWARDS_OFFSET, 0, Math.toRadians(90)) // back
        ));

        leftEncoder = (ExpansionHubMotor) encLeft;
        rightEncoder = (ExpansionHubMotor) encRight;
        backEncoder = (ExpansionHubMotor) encBack;
        expansionHub = hardwareMap.get(ExpansionHubEx.class, configs.expansionHubSistemeName);
        leftEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        backEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(-BACKWARDS_OFFSET, 0, Math.toRadians(90)) // back
        ));

        leftEncoder = (ExpansionHubMotor) hardwareMap.dcMotor.get(configs.encStName);
        rightEncoder = (ExpansionHubMotor) hardwareMap.dcMotor.get(configs.encDrName);
        backEncoder = (ExpansionHubMotor) hardwareMap.dcMotor.get(configs.encSpName);
        expansionHub = hardwareMap.get(ExpansionHubEx.class, configs.expansionHubSistemeName);
        leftEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        backEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public static double encoderTicksToCms(int ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        bulkData = expansionHub.getBulkInputData();
        return Arrays.asList(
                encoderTicksToCms(bulkData.getMotorCurrentPosition(leftEncoder)),
                encoderTicksToCms(bulkData.getMotorCurrentPosition(rightEncoder)),
                encoderTicksToCms(bulkData.getMotorCurrentPosition(backEncoder))
        );
    }
}
