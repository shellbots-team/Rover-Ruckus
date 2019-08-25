package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by wave on 10/16/2018.
 */

@Autonomous
@Disabled
public class DepoNotLanderAuto extends LinearOpMode {

    // Declare motors/servos
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor armMotor;
    private Servo handServo;
    private Servo markerServo;

    @Override
    public void runOpMode() {

        // Initialize motors/servos
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        handServo = hardwareMap.get(Servo.class, "handServo");
        markerServo = hardwareMap.get(Servo.class, "markerServo");

        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        waitForStart();


        // Step 1: Moving to 3 elements
        telemetry.addData("Status", "Moving to Elements");
        telemetry.update();
        move(direction.FORWARD, 1);
        sleep(1000);
        stopMotor();

        // Step 2: Drop marker
        telemetry.addData("Status", "Dropping Marker");
        telemetry.update();
        dropMarker();
        sleep(250);
        stopMotor();

        // Step 3: Moving away from elements
        telemetry.addData("Status", "Moving away from elements");
        telemetry.update();
        move(direction.BACKWARD, 1);
        sleep(100);
        stopMotor();

        // Step 4: Raise marker servo
        telemetry.addData("Status", "Raising marker servo");
        telemetry.update();
        raiseMarker();
        sleep(250);
        stopMotor();

        // Step 5: Stop
        telemetry.addData("Status", "Finished");
        telemetry.update();

    }

    enum direction {
        FORWARD,
        BACKWARD,
        TURN_CLOCKWISE,
        TURN_COUNTERCLOCKWISE
    }

    private void move(direction dir, double power) {

        if (dir == direction.FORWARD) { // Set motors to move forward
            leftMotor.setPower(-power);
            rightMotor.setPower(power);
        } else if (dir == direction.BACKWARD) { // Set motors to move backward
            leftMotor.setPower(power);
            rightMotor.setPower(-power);
        } else if (dir == direction.TURN_CLOCKWISE) { // Set motors to turn clockwise
            leftMotor.setPower(-power);
            rightMotor.setPower(-power);
        } else if (dir == direction.TURN_COUNTERCLOCKWISE) { // Set motors to turn counter-clockwise
            leftMotor.setPower(power);
            rightMotor.setPower(power);
        }
    }

    private void stopMotor() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        armMotor.setPower(0);
    }

    private void openHand() {
        handServo.setPosition(0.85);
    }
    private void closeHand() {
        handServo.setPosition(0.05);
    }

    private void dropMarker() {
        markerServo.setPosition(0.2275);
    }

    private void raiseMarker() {
        markerServo.setPosition(0.82825);
    }

}