package com.qualcomm.ftcrobotcontroller.QuantumMechanics;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
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


    Servo leftShield;
    Servo rightShield;
    static final double SHIELD_UP = 0;
    static final double SHIELD_DOWN = .9;

    Servo aim;
    double doorRV = 0;
    double doorLV = .7;
    Servo hook;
    boolean armPressed;
    boolean armState;
    double armPos1 =.2;
    double armPos2 =.6;
    //values for the pullup
    double hangPos = .2;
    double maxChangeRate = .01/2;

    static final double INIT_LEFT_POS = 0;
    static final double INIT_RIGHT_POS = .7;

    boolean shieldPressed;
    boolean shieldDown = false;

    int encoderPos;

    boolean dab = false;

    //Nathaniel's Sensor Variables:
    double orientation = 0;
    boolean hasStarted = false;
    double prevHeading = 0;
    long systemTime = System.nanoTime();
    long prevTime = System.nanoTime();

    boolean gyroBeenInit = false;
    AdafruitIMU gyro;

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


        leftShield = hardwareMap.servo.get("leftShield");
        rightShield = hardwareMap.servo.get("rightShield");

        moveShields(SHIELD_UP);
        shieldPressed = false;


                //getting servos
        aim = hardwareMap.servo.get("pullupS");
        hook = hardwareMap.servo.get("arm");
        hook.setPosition(armPos2);
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

        initGyro();
    }

    void initGyro() {
        gyroBeenInit = true;
        systemTime = System.nanoTime();
        prevTime = systemTime;
        try {
            gyro = new AdafruitIMU(hardwareMap, "bno055"
                    , (byte) (AdafruitIMU.BNO055_ADDRESS_A * 2)//By convention the FTC SDK always does 8-bit I2C bus
                    , (byte) AdafruitIMU.OPERATION_MODE_IMU);
        } catch (RobotCoreException e) {
            Log.i("FtcRobotController", "Exception: " + e.getMessage());
        }

        systemTime = System.nanoTime();
        gyro.startIMU();//Set up the IMU as needed for a continual stream of I2C reads.
        telemetry.addData("FtcRobotController", "IMU Start method finished in: "
                + (-(systemTime - (systemTime = System.nanoTime()))) + " ns.");
    }

    void moveShields(double pos) {
        pos = scaleServo(pos);
        leftShield.setPosition(pos);
        rightShield.setPosition(1.0-pos);
    }

    double sign(double d) {
        if (d >= 0) return 1;
        else return -1;
    }
    static final double MIN_POWER = .5;
    static final double SLOW_MODE = .3;
    static final double RAMP_SPEED = .85;

    boolean moveConveyor = false;
    int conveyorTicks = 0;
    double conveyorPow = 0.0;
    @Override
    public void loop() {
        if (gyroBeenInit) {
            updateHeading();
            if (gamepad1.start || gamepad2.start) {
                gyro.offsetsInitialized = false;
            }
        }

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
         * Shields
         */
        if (gamepad1.x)
            shieldDown = false;
        else if (gamepad1.b)
            shieldDown = true;

        if (!shieldDown || onRamp())
            moveShields(SHIELD_UP);
        else
            moveShields(SHIELD_DOWN);

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

        if (gamepad2.left_bumper) {
            leftZipPos = INIT_LEFT_POS;
        }
        else if (gamepad2.right_bumper) {
            rightZipPos = INIT_RIGHT_POS;
        }
        else if (gamepad2.left_trigger > .1) {
            leftZipPos = INIT_LEFT_POS;
            rightZipPos = INIT_RIGHT_POS;
        }
        else if (gamepad2.dpad_left) {
            leftZipPos += .01;
        }
        else if (gamepad2.dpad_right) {
            rightZipPos -= .01;
        }
        leftZipPos = scaleServo(leftZipPos);
        rightZipPos = scaleServo(rightZipPos);
        leftZip.setPosition(leftZipPos);
        rightZip.setPosition(rightZipPos);

        /*
         * Pullup
         */
        //Do a pullup
        double pup = gamepad2.right_stick_y;
        if(Math.abs(pup) > .5) {
            pullup.setPower(pup);
        } else {
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
//        cspeed = sign(cspeed) * Math.pow(cspeed, 4);
        cspeed = Math.abs(cspeed) > .2 ? cspeed : 0.0;
        conveyor.setPower(cspeed/3);

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

//        telemetry.addData("Encoder distance", mL1.getCurrentPosition() - encoderPos);
        telemetry.addData("Left Motor Power", mL1.getPower());
        telemetry.addData("Right Motor Power", mR1.getPower());
        telemetry.addData("Heading", orientation);
        telemetry.addData("Pitch", rollAngle[0]);
        telemetry.addData("On Ramp?", onRamp());
    }

    boolean onRamp() {
        return gyroBeenInit && rollAngle[0] < -15.0;
    }

    double scaleServo(double d) {
        if (d > 1.0)
            return 1;
        else if (d < 0.0)
            return 0;
        else
            return d;
    }

    //The following arrays contain both the Euler angles reported by the IMU (indices = 0) AND the
    // Tait-Bryan angles calculated from the 4 components of the quaternion vector (indices = 1)
    static volatile double[] rollAngle = new double[2], pitchAngle = new double[2], yawAngle = new double[2], accs = new double[3];
    public void updateHeading() {
        //Update gyro values
        if (gyroBeenInit) {
            gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
            prevHeading = orientation;
            orientation = yawAngle[0];
        }
    }
}
