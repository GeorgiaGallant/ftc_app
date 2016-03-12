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

    Servo pullupS;
    double doorRV = 0;
    double doorLV = .7;
    Servo arm;
    boolean armPressed;
    boolean armState;
    double armPos1 =.2;
    double armPos2 =.8;
    //values for the pullup
    double hangPos = .5;
    double maxChangeRate = .01;

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

        //getting servos
        pullupS = hardwareMap.servo.get("pullupS");
        arm = hardwareMap.servo.get("arm");

        //setting motor directions
        mR1.setDirection(DcMotor.Direction.FORWARD);
        mL1.setDirection(DcMotor.Direction.REVERSE);
        elevator.setDirection(DcMotor.Direction.REVERSE);
        pullup.setDirection(DcMotor.Direction.REVERSE);
        conveyor.setDirection(DcMotor.Direction.FORWARD);
    }

    double sign(double d) {
        if (d >= 0) return 1;
        else return -1;
    }
    static final double MIN_POWER = .5;

    @Override
    public void loop() {
        double throttle = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        throttle = sign(throttle)*Math.pow(throttle,2);
        turn = sign(turn)*Math.pow(throttle,2);

        if (Math.abs(turn) > MIN_POWER || Math.abs(throttle) > MIN_POWER) {
            if (Math.abs(turn) > Math.abs(throttle)) {
                mL1.setPower(turn);
                mR1.setPower(-turn);
            } else {
                mL1.setPower(throttle);
                mR1.setPower(throttle);
            }
        }

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

        //manual pullupS control
        if(hangPos>.95) hangPos=.95;
        if(hangPos<.05) hangPos=.05;
        if (gamepad2.dpad_down) hangPos += maxChangeRate;
        else if (gamepad2.dpad_up) hangPos -= maxChangeRate;
        pullupS.setPosition(hangPos);

        //conveyerbelt control slow
        if (gamepad2.dpad_right){
            conveyor.setPower(.4);
        }
        else if (gamepad2.dpad_left){
            conveyor.setPower(-.4);
        }
        else{
            conveyor.setPower(0);
        }
        if(gamepad1.y) armPressed = true;
        else if(armPressed) {
            armState = !armState;
            armPressed = false;
        }
        if(armState) arm.setPosition(armPos1);
        else arm.setPosition(armPos2);

        telemetry.addData("Encoder distance", mR1.getCurrentPosition() - encoderPos);
    }
}
