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
public class CraterSide extends LinearOpMode {

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

        // Setting status to "Ready to run"
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        // Waiting until user presses start
        waitForStart();

        // Step 1: Lowering robot from lander
        telemetry.addData("Status", "Lowering");
        telemetry.update();
        armMotor.setPower(1.0);
        sleep(2300);
        stopMotor();

        // Step 2: Releasing Hand
        telemetry.addData("Status", "Releasing Hand");
        telemetry.update();
        openHand();
        sleep(1000);

        // Step 3: Move backwards
        telemetry.addData("Status", "Moving Backwards");
        telemetry.update();
        move(direction.BACKWARD, 1);
        sleep(500);
        stopMotor();

        // Step 4: Turn around
        telemetry.addData("Status", "Turning");
        telemetry.update();
        move(direction.TURN_CLOCKWISE, 1);
        sleep(1100); // 1000 for robots left element
        stopMotor();

        // Step 5: Moving to 3 elements
        telemetry.addData("Status", "Moving to Elements");
        telemetry.update();
        move(direction.FORWARD, 1);
        sleep(1050);
        stopMotor();

        // Step 6: Moving away from elements
        telemetry.addData("Status", "Moving away from elements");
        telemetry.update();
        move(direction.BACKWARD, 1);
        sleep(300);
        stopMotor();

        // Step 7: Turning to wall
        telemetry.addData("Status", "Turning to wall");
        telemetry.update();
        move(direction.TURN_CLOCKWISE, 1);
        sleep(700);
        stopMotor();

        // Step 8: Moving to wall
        telemetry.addData("Status", "Moving to wall");
        telemetry.update();
        move(direction.FORWARD, 1);
        sleep(1925);
        stopMotor();

        // Step 9: Fixing turn
        telemetry.addData("Status", "Fixing turn");
        telemetry.update();
        move(direction.TURN_CLOCKWISE, 1);
        sleep(250);
        stopMotor();

        // Step 10: Moving to depo
        telemetry.addData("Status", "Moving to depo");
        telemetry.update();
        move(direction.FORWARD, 1);
        sleep(1550);
        stopMotor();

        // Step 11: Dropping Marker
        telemetry.addData("Status", "Dropping marker");
        telemetry.update();
        dropMarker();
        sleep(250);
        stopMotor();

        // Step 12: Moving backwards to crater
        telemetry.addData("Status", "Moving backwards");
        telemetry.update();
        move(direction.BACKWARD, 1);
        sleep(500);
        stopMotor();

        // Step 13: Raising marker arm
        telemetry.addData("Status", "Raising marker arm");
        telemetry.update();
        raiseMarker();
        sleep(250);
        stopMotor();

        // Step 14: Moving onto crater
        telemetry.addData("Status", "Moving onto crater");
        telemetry.update();
        move(direction.BACKWARD, 1);
        sleep(2750);
        stopMotor();

        // Step 15: Finished
        telemetry.addData("Status", "Finished");
        telemetry.update();

    }

    enum direction {
        FORWARD,
        BACKWARD,
        TURN_CLOCKWISE,
        TURN_COUNTERCLOCKWISE
    }

    /**
     * Takes in a direction and a power then sets the motors to the correct speed based on these variables
     * @param dir the direction for the robot to move
     * @param power the power to give the motors
     */

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

    /**
     * Setting all motors speed to 0
     */

    private void stopMotor() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        armMotor.setPower(0);
    }

    private void dropMarker() {
        markerServo.setPosition(0.2275);
    }

    private void raiseMarker() {
        markerServo.setPosition(0.82825);
    }

    private void openHand() {
        handServo.setPosition(0.85);
    }

    private void closeHand() {
        handServo.setPosition(0.05);
    }

}