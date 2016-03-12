package com.qualcomm.ftcrobotcontroller.QuantumMechanics;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class TeleOp extends OpMode {
    DcMotor mL1;
    DcMotor mR1;
    DcMotor elevator;
    DcMotor pullup;
    DcMotor conveyor;
    DcMotor nom;

    Servo leftZip;
    Servo rightZip;
    double leftZipPos;
    double rightZipPos;

    Servo aim;
    double doorRV = 0;
    double doorLV = .7;
    Servo hook;
    boolean armPressed;
    boolean armState;
    double armPos1 =.2;
    double armPos2 =.6;
    //values for the pullup
    double hangPos = .5;
    double maxChangeRate = .01/2;

    static final double INIT_LEFT_POS = 0;
    static final double INIT_RIGHT_POS = 1;

    int encoderPos;

    @Override
    public void init() {
        //getting motors
        mL1 = hardwareMap.dcMotor.get("mL1");
        mR1 = hardwareMap.dcMotor.get("mR1");
        encoderPos = mR1.getCurrentPosition();
        elevator = hardwareMap.dcMotor.get("nom");
        pullup = hardwareMap.dcMotor.get("pullup");
        conveyor = hardwareMap.dcMotor.get("conveyor");
        nom = hardwareMap.dcMotor.get("nomF");
        leftZip = hardwareMap.servo.get("leftZip");
        rightZip = hardwareMap.servo.get("rightZip");


        //getting servos
        aim = hardwareMap.servo.get("pullupS");
        hook = hardwareMap.servo.get("arm");

        //setting motor directions
        mL1.setDirection(DcMotor.Direction.FORWARD);
        mR1.setDirection(DcMotor.Direction.REVERSE);
        elevator.setDirection(DcMotor.Direction.REVERSE);
        pullup.setDirection(DcMotor.Direction.REVERSE);
        conveyor.setDirection(DcMotor.Direction.FORWARD);

        leftZipPos = INIT_LEFT_POS;
        rightZipPos = INIT_RIGHT_POS;

        leftZip.setPosition(leftZipPos);
        rightZip.setPosition(rightZipPos);
    }

    double sign(double d) {
        if (d >= 0) return 1;
        else return -1;
    }
    static final double MIN_POWER = .5;
    static final double SLOW_MODE = .3;
    static final double RAMP_SPEED = .8;
    @Override
    public void loop() {
        /*
         * Driving
         */
        double throttle = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        throttle = sign(throttle)*Math.pow(throttle,2);

        if (gamepad1.dpad_up) {
            mL1.setPower(RAMP_SPEED);
            mR1.setPower(RAMP_SPEED);
        }
        else if (gamepad1.dpad_down) {
            mL1.setPower(-RAMP_SPEED);
            mR1.setPower(-RAMP_SPEED);
        }
        else if (gamepad1.y) {
            mL1.setPower(SLOW_MODE);
            mR1.setPower(SLOW_MODE);
        }
        else if (gamepad1.a) {
            mL1.setPower(-SLOW_MODE);
            mR1.setPower(-SLOW_MODE);
        }
        else if (Math.abs(turn) > MIN_POWER || Math.abs(throttle) > MIN_POWER) {
            if (Math.abs(turn) > Math.abs(throttle)) {
                mL1.setPower(turn);
                mR1.setPower(-turn);
            } else {
                mL1.setPower(throttle);
                mR1.setPower(throttle);
            }
        } else {
            mL1.setPower(0);
            mR1.setPower(0);
        }

        /*
         * Pickup
         */

        if(gamepad2.a) {
            elevator.setPower(1);
            nom.setPower(1);
        }
        else if(gamepad2.y) {
            nom.setPower(-1);
        }
        else if (gamepad2.x) {
            elevator.setPower(-1);
        }
        else if (gamepad2.b) {
            elevator.setPower(-1);
            nom.setPower(-1);
        }
        else{
            elevator.setPower(0);
            nom.setPower(0);
        }

        /*
         * Ziplines
         */

        if (gamepad2.dpad_up) {
            rightZipPos = INIT_RIGHT_POS;
            leftZipPos = INIT_LEFT_POS;
        }
        else if (gamepad2.dpad_left) {
            leftZipPos++;
        }
        else if (gamepad2.dpad_right) {
            rightZipPos--;
        }
        leftZipPos = scaleServo(leftZipPos);
        rightZipPos = scaleServo(rightZipPos);
        leftZip.setPosition(leftZipPos);
        rightZip.setPosition(rightZipPos);

        /*
         * Pullup
         */
        //Do a pullup
        if(gamepad2.right_bumper) {
            pullup.setPower(-.7);
        }
        else if(gamepad2.left_bumper) {
            pullup.setPower(.7);
        }
        else{
            pullup.setPower(0);
        }

        //manual aim control
        if(hangPos>.95) hangPos=.95;
        if(hangPos<.05) hangPos=.05;
        if (gamepad2.dpad_down) hangPos += maxChangeRate;
        else if (gamepad2.dpad_up) hangPos -= maxChangeRate;
        aim.setPosition(hangPos);

        /*
         * Placement
         */
        double cspeed = gamepad2.left_stick_x;
        cspeed = sign(cspeed) * Math.pow(cspeed, 4);
        cspeed = Math.abs(cspeed) > .1 ? cspeed : 0.0;
        conveyor.setPower(cspeed/2);

        /*
         * Hook
         */
        if(gamepad1.right_bumper) armPressed = true;
        else if(armPressed) {
            armState = !armState;
            armPressed = false;
        }
        if(armState) hook.setPosition(armPos1);
        else hook.setPosition(armPos2);

        telemetry.addData("Encoder distance", mR1.getCurrentPosition() - encoderPos);
    }

    double scaleServo(double d) {
        if (d > 1.0)
            return 1;
        else if (d < 0.0)
            return 0;
        else
            return d;
    }
}
