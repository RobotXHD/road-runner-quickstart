package org.firstinspires.ftc.teamcode.drive.localizer;

import android.support.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.opModes.configs;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 * Note: this could be optimized significantly with REV bulk reads
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 4000;
    public static double WHEEL_RADIUS = 2; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 7.97244094488 * 2; // in; distance between the left and right wheels 405/2.54
    public static double BACKWARDS_OFFSET = 4.92125984251; // in; offset of the lateral wheel

    public ExpansionHubEx expansionHub;
    public RevBulkData bulkData;

    private ExpansionHubMotor leftEncoder, rightEncoder, backEncoder;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(-BACKWARDS_OFFSET, 0, Math.toRadians(90)) // back
        ));

        leftEncoder = (ExpansionHubMotor) hardwareMap.dcMotor.get(configs.encStName);
        rightEncoder = (ExpansionHubMotor) hardwareMap.dcMotor.get(configs.encDrName);
        backEncoder = (ExpansionHubMotor) hardwareMap.dcMotor.get(configs.encSpName);
        expansionHub = hardwareMap.get(ExpansionHubEx.class, configs.expansionHubOdometrieName);
    }

    public static double encoderTicksToInches(int ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        bulkData = expansionHub.getBulkInputData();
        return Arrays.asList(
                encoderTicksToInches(bulkData.getMotorCurrentPosition(leftEncoder)),
                encoderTicksToInches(bulkData.getMotorCurrentPosition(rightEncoder)),
                encoderTicksToInches(bulkData.getMotorCurrentPosition(backEncoder))
        );
    }
}
