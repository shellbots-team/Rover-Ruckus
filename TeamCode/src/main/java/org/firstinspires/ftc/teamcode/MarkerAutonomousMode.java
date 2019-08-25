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
public class MarkerAutonomousMode extends LinearOpMode {

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

        // Step 1: Lowering robot from lander
        telemetry.addData("Status", "Lowering");
        telemetry.update();
        armMotor.setPower(0.5);
        sleep(3100);
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
        sleep(1050); // 1000 for robots left element
        stopMotor();

        // Step 5: Moving to 3 elements
        telemetry.addData("Status", "Moving to Elements");
        telemetry.update();
        move(direction.FORWARD, 1);
        sleep(2050);
        stopMotor();

        // Step 6: Dropping marker
        telemetry.addData("Status", "Dropping marker");
        telemetry.update();
        dropMarker();
        sleep(250);
        stopMotor();

        // Step 7: Moving away
        telemetry.addData("Status", "Moving from marker");
        telemetry.update();
        move(direction.BACKWARD, 1);
        sleep(1650);
        stopMotor();

        // Step 8: Raise marker servo
        telemetry.addData("Status", "Reraising marker servo");
        telemetry.update();
        raiseMarker();
        sleep(100);
        stopMotor();

        //Step 9: Turning to side
        telemetry.addData("Status", "Turning to side");
        telemetry.update();
        move(direction.TURN_CLOCKWISE, 1);
        sleep(500);
        stopMotor();

        //Step 10: Moving to side
        telemetry.addData("Status", "Moving to side");
        telemetry.update();
        move(direction.FORWARD, 1);
        sleep(1600);
        stopMotor();

        //Step 11: Adjust Turning
        telemetry.addData("Status", "Adjusting Turn");
        telemetry.update();
        move(direction.TURN_CLOCKWISE, 1);
        sleep(1600);
        stopMotor();

        //Step 12: Move Into Crater
        telemetry.addData("Status", "Moving into crater");
        telemetry.update();
        move(direction.BACKWARD, 1);
        sleep(1500);
        stopMotor();

        // Step 13: Stop
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