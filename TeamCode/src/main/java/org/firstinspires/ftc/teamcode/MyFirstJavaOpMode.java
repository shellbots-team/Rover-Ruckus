package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by shell on 10/13/2018.
 */

@TeleOp

public class MyFirstJavaOpMode extends LinearOpMode {

    // Declare motors/servos/variables
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor armMotor;
    private Servo handServo;
    private Servo markerServo;
    private double rightPower;
    private double leftPower;
    private String armMoving;
    private String handPosition;
    private double speed;
    private String speedName;
    private String markerPos;


    @Override
    public void runOpMode() {

        // Initialize motors/servos
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        handServo = hardwareMap.get(Servo.class, "handServo");
        markerServo = hardwareMap.get(Servo.class, "markerServo");
        handPosition = "open";
        speed = 1;
        speedName = "Full";
        markerPos = "up";

        // Set status
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // Do this while program is running
        while (opModeIsActive()) {

            if (this.gamepad1.x) {
                markerPos = "down";
                markerServo.setPosition(0.9);
            } else if (this.gamepad1.b) {
                //markerPos = "down";
                //markerServo.setPosition(0.2275);
            } else if (this.gamepad1.y) {
                //raiseArm();
            } else if (this.gamepad1.a) {
                //lowerArm();
                markerPos = "up";
                markerServo.setPosition(0.3);
            }

            // Set the hand servo to open or closed
            if (this.gamepad1.dpad_left) {
                openHand();
                handPosition = "open";
            } else if (this.gamepad1.dpad_right) {
                closeHand();
                handPosition = "closed";
            }

            // Set speed based on trigger clicks
            if (this.gamepad1.right_trigger > 0.5) { // When right trigger is clicked
                speed = 1;
                speedName = "Full";
            } else if (this.gamepad1.left_trigger > 0.5) { // When left trigger is clicked
                speed = 0.5;
                speedName = "Half";
            }

            // Set the arm motor to up/down
            if (this.gamepad1.dpad_up == this.gamepad1.dpad_down) { // Both or neither are being pushed
                armMotor.setPower(0.0);
                armMoving = "not moving";
            } else if (this.gamepad1.dpad_up) { // If only up is pushed, move up
                armMotor.setPower(-1.0 * speed);
                armMoving = "moving up";
            } else if (this.gamepad1.dpad_down) { // If only down is pushed, move down
                armMotor.setPower(1.0 * speed);
                armMoving = "moving down";
            }

            // Set variables to analog stick (-1 to 1) then multiply by speed (0.5 or 1)
            leftPower = this.gamepad1.right_stick_y * speed;
            rightPower = -this.gamepad1.left_stick_y * speed;

            // Set motor power to variable
            powerMotors();

            // Display information about the motors
            telemetry.addData("Speed", speedName);
            telemetry.addData("Right Motor Power", leftMotor.getPower());
            telemetry.addData("Left Motor Power", rightMotor.getPower());
            telemetry.addData("Arm is", armMoving);
            telemetry.addData("Hand is ", handPosition);
            telemetry.addData("Status", "Running");
            telemetry.addData("Marker Position", markerServo.getPosition());
            telemetry.update();
        }
        stopMotors();
    }

    /**
     * Sets The Power Of All Motors (Left, Right, And Arm) To 0
     */

    private void stopMotors() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        armMotor.setPower(0);
    }

    private void powerMotors() {
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
    }

    private void openHand() {
        handServo.setPosition(0.9);
    }

    private void closeHand() {
        handServo.setPosition(0.05);
    }

    private void raiseArm() {
        singleMotorEncoder(armMotor, 7025);
    }

    private void lowerArm() {
        singleMotorEncoder(armMotor, -7025);
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
}