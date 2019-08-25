package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by shell on 11/17/2018.
 */

@Autonomous
@Disabled
public class onCrater extends LinearOpMode { // Outdated Code

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

        updateTelemetry("Lowering");
        singleMotorEncoder(armMotor, 7025);

        updateTelemetry("Releasing Hand");
        openHand();
        sleep(400);

        updateTelemetry("Moving Backwards");
        move(1, 1, -20, -20);

        updateTelemetry("Stopping");
        stopMotors();
    }

    /**
     * Prints a message to the telemetry under "Status" and updates it
     *
     * @param msg Message to be printed to telemetry
     */

    private void updateTelemetry(String msg) {
        // Add the data to telemetry
        telemetry.addData("Status", msg);
        // Update telemetry
        telemetry.update();
    }

    /**
     * Runs the robot wheels a specified distance
     *
     * @param leftPower     the power for the left motor (0.0 - 1.0)
     * @param rightPower    the power for the right motor (0.0 - 1.0)
     * @param leftPosition  the position for the left motor to travel to (inches)
     * @param rightPosition the position for the right motor to travel to (inches)
     */

    private void move(int leftPower, int rightPower, double leftPosition, double rightPosition) {
        // Converts inches given to ticks
        leftPosition = inchesToTicks(leftPosition);
        rightPosition = inchesToTicks(rightPosition);

        // Reverse right position (because motor is backwards)
        rightPosition = -rightPosition;

        // Reset the tick count on encoders
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Sets mode to RTP (run to position)
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set power to given powers
        leftMotor.setPower(1);
        rightMotor.setPower(1);

        // Sets target position
        leftMotor.setTargetPosition((int) leftPosition);
        rightMotor.setTargetPosition((int) rightPosition);

        // Create booleans if running
        boolean leftRunning = true;
        boolean rightRunning = true;

        // While either motor is running and op mode is active
        while (opModeIsActive() && leftRunning && rightRunning) {
            // Add data to the telemetry about right and left motor then update it
            telemetry.addData("Right: position", rightMotor.getCurrentPosition());
            telemetry.addData("Right: target", rightMotor.getTargetPosition());
            telemetry.addData("Right: busy status", rightMotor.isBusy());

            telemetry.addData("Left: position", leftMotor.getCurrentPosition());
            telemetry.addData("Left: target", leftMotor.getTargetPosition());
            telemetry.addData("Left: busy status", leftMotor.isBusy());

            telemetry.update();

            // If left/right motor is at target position
            // Set motor's power to 0 and boolean (leftRunning, rightRunning) to false;
            if (Math.abs(leftMotor.getCurrentPosition()) >= Math.abs(leftPosition)) {
                leftMotor.setPower(0);
                leftRunning = false;
            } else {
                leftMotor.setPower(1);
            }
            if (Math.abs(rightMotor.getCurrentPosition()) >= Math.abs(rightPosition)) {
                rightMotor.setPower(0);
                rightRunning = false;
            } else {
                rightMotor.setPower(1);
            }
        }

        // When over set both powers to 0
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    /**
     * Runs a single motor to a set amount of ticks
     *
     * @param currentMotor The motor to run
     * @param position     The position (in ticks) to run to
     */

    private void singleMotorEncoder(DcMotor currentMotor, int position) {
        // Reset tick count on encoder and set mode to run to position
        currentMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        currentMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set power to 1 and motor's target position to position passed
        currentMotor.setPower(1);
        currentMotor.setTargetPosition(position);

        // While motor is running and op mode is active
        while (opModeIsActive() && currentMotor.isBusy()) {
            // Add data to the telemetry and update
            telemetry.addData("position", currentMotor.getCurrentPosition());
            telemetry.addData("busy status", currentMotor.isBusy());
            telemetry.addData("target", currentMotor.getTargetPosition());
            telemetry.update();
            idle();
        }

        // Update the data after motor is done running
        telemetry.addData("position", currentMotor.getCurrentPosition());
        telemetry.addData("busy status", currentMotor.isBusy());
        telemetry.addData("target", currentMotor.getTargetPosition());
        telemetry.update();

        // Set power to 0
        currentMotor.setPower(0);
    }

    /**
     * Sets The Power Of All Motors (Left, Right, And Arm) To 0
     */

    private void stopMotors() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        armMotor.setPower(0);
    }

    /**
     * Lowers the servo the marker is set on
     */

    private void dropMarker() {
        markerServo.setPosition(0.24);
    }

    /**
     * Raise the servo the marker is set on
     */

    private void raiseMarker() {
        markerServo.setPosition(0.82825);
    }

    /**
     * Sets the hand servo to the open position
     */

    private void openHand() {
        handServo.setPosition(0.85);
    }

    /**
     * Sets the hand servo to a closed position
     */

    private void closeHand() {
        handServo.setPosition(0.05);
    }

    /**
     * Converts inches to an amount of ticks
     *
     * @param inches Amount of inches to convert to ticks
     * @return Amount of ticks
     */

    private int inchesToTicks(double inches) {
        // Divide inches by 0.0062 and cast to integer
        return (int) (inches / 0.0062);
    }
}
