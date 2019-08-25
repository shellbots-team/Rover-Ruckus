package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

/**
 * Created by wave on 12/12/2018.
 */

@Autonomous

public class turnAround extends LinearOpMode {

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

        waitForStart();

        move(1, 1, 50, -50);
        stopMotors();

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

    /**
     * Sets The Power Of All Motors (Left, Right, And Arm) To 0
     */

    private void stopMotors() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        armMotor.setPower(0);
    }

    private void move(int leftPower, int rightPower, double leftPosition, double rightPosition) {
        sleep(50);

        // Converts inches given to ticks
        leftPosition = inchesToTicks(leftPosition);
        rightPosition = inchesToTicks(rightPosition);

        // Reverse left position (because motor is backwards)
        leftPosition = -leftPosition;

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
        sleep(50);
        // When over set both powers to 0
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        sleep(50);
    }
}
