package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by wave on 11/18/2018.
 */

@TeleOp
@Disabled
public class colorSensorTest extends LinearOpMode {
    private DistanceSensor dist;

    @Override
    public void runOpMode() {

        dist = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        waitForStart();

        telemetry.addData("Status", "Not run yet");
        telemetry.update();

        while(opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.addData("Distance", dist.getDistance(DistanceUnit.INCH));
            String d = String.valueOf(dist.getDistance(DistanceUnit.INCH));
            telemetry.addData("Other Distance", d);
            telemetry.update();
        }
    }
}
