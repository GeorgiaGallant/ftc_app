package com.qualcomm.ftcrobotcontroller.QuantumMechanics;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */

//Nathaniel is my Captain forever
//I pledge allegiance to the flag
//of Atomic Theory
//and to the Republic for which it stands
//One team
//Under God
//With Liberty and Justice for All

public class TeleOp extends OpMode {
    DcMotor mL1;
    DcMotor mR1;
    DcMotor elevator;
    DcMotor pullup;
    DcMotor conveyor;
    DcMotor nom;
    int encoderPos;

    VoltageSensor battery;

    Servo leftZip;
    Servo rightZip;
    double leftZipPos;
    double rightZipPos;

    Servo leftShield;
    Servo rightShield;

    boolean shieldPressed;
    boolean shieldDown = false;

    Servo leftDoor;
    Servo rightDoor;
    static final double DOOR_UP = .4;
    static final double DOOR_DOWN = 1.0;

    Servo aim;
    Servo hook;
    boolean armPressed;
    boolean armState;
    double armPos1 = 0;
    double armPos2 =.6;
    //values for the pullup
    double hangPos = .2;

    double orientation = 0;
    double pitch = 0;
    double prevHeading = 0;
    long systemTime = System.nanoTime();
    long prevTime = System.nanoTime();

    boolean gyroBeenInit = false;
    boolean useGyro = true;
    AdafruitIMU gyro;

    boolean isTipping = false;
    boolean pullupEngaged = false;
    /*
     * Constants
     */
    static final double AIM_SPEED = .01/2;

    static final double INIT_LEFT_POS = 0;
    static final double INIT_RIGHT_POS = .7;

    static final double LEFT_SHIELD_UP = 0;
    static final double LEFT_SHIELD_DOWN = .93;

    static final double RIGHT_SHIELD_UP = 1;
    static final double RIGHT_SHIELD_DOWN = .05;

    static final double MIN_POWER = .5;
    static final double SLOW_MODE = .3;
    static final double RAMP_SPEED = .85;
    static final double STUMBLE_SPEED  = 1.0;

    static final double FLOOR_THRESHOLD = 5;
    static final double RAMP_MIN_ANGLE = 15;
    static final double TIP_MIN_ANGLE = 50;

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

        battery = hardwareMap.voltageSensor.get("Drivetrain");
        leftDoor = hardwareMap.servo.get("leftDoor");
        rightDoor = hardwareMap.servo.get("rightDoor");


        leftShield = hardwareMap.servo.get("leftShield");
        rightShield = hardwareMap.servo.get("rightShield");

        moveShields(LEFT_SHIELD_UP, RIGHT_SHIELD_UP);
        shieldPressed = false;


                //getting servos
        aim = hardwareMap.servo.get("pullupS");
        hook = hardwareMap.servo.get("arm");
        hook.setPosition(armPos2);
        //setting motor directions
        mL1.setDirection(DcMotor.Direction.FORWARD);
        mR1.setDirection(DcMotor.Direction.REVERSE);
        elevator.setDirection(DcMotor.Direction.REVERSE);
        pullup.setDirection(DcMotor.Direction.FORWARD);
        conveyor.setDirection(DcMotor.Direction.FORWARD);

        leftZipPos = INIT_LEFT_POS;
        rightZipPos = INIT_RIGHT_POS;

        leftZip.setPosition(leftZipPos);
        rightZip.setPosition(rightZipPos);

//        initGyro();
    }

    @Override
    public void loop() {
        if (gyroBeenInit && useGyro) {
            updateHeading();
            if (gamepad1.start || gamepad2.start) {
                gyro.offsetsInitialized = false;
            }
        }
        if (gamepad1.start || gamepad2.start) {
            pullupEngaged = false;
        }

        pullupEngaged = gamepad2.right_trigger > .5;

        if (gamepad1.right_trigger > .5)
            useGyro = false;
        if (gamepad1.left_trigger > .5)
            useGyro = true;

        /*
         * Anti-tipping
         * After the robot tips by more than 30 degrees, drive backwards
         * until we're on the floor. Override manual control during this time.
         */
        if (tipping())
            isTipping = true;

        if (useGyro && isTipping) {
            /* if we're tipping forward, go backwards, and vice versa.
               At the moment gyro is mounted backwards, so negative pitch (pitching forward)
               means we'll move backwards (negative power to the motors)
            */
            mL1.setPower(STUMBLE_SPEED*sign(pitch));
            mR1.setPower(STUMBLE_SPEED*sign(pitch));
            leftZipPos = INIT_LEFT_POS;
            rightZipPos = INIT_RIGHT_POS;
            if (onFloor())
                isTipping = false;
        } else {
            /*
             * Driving
             */
            double throttle = gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            throttle = sign(throttle) * Math.pow(throttle, 2);

            if (gamepad1.dpad_up) {
                mL1.setPower(RAMP_SPEED);
                mR1.setPower(RAMP_SPEED);
            } else if (gamepad1.dpad_down) {
                mL1.setPower(-RAMP_SPEED);
                mR1.setPower(-RAMP_SPEED);
            } else if (gamepad1.y) {
                mL1.setPower(SLOW_MODE);
                mR1.setPower(SLOW_MODE);
            } else if (gamepad1.a) {
                mL1.setPower(-SLOW_MODE);
                mR1.setPower(-SLOW_MODE);
            } else if (Math.abs(turn) > MIN_POWER || Math.abs(throttle) > MIN_POWER) {
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
        }

        /*
         * Shields
         */
        if (gamepad1.x)
            shieldDown = false;
        else if (gamepad1.b)
            shieldDown = true;

        if (!shieldDown || onRamp())
            moveShields(LEFT_SHIELD_UP, RIGHT_SHIELD_UP);
        else
            moveShields(LEFT_SHIELD_DOWN, RIGHT_SHIELD_DOWN);

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

        if (gamepad2.dpad_left)
            leftZipPos += .01;
        else if (gamepad2.dpad_right)
            rightZipPos -= .01;
        else if (pullingUp()) { // if we're pulling up and no button is pressed, reset zipliners.
            leftZipPos = INIT_LEFT_POS;
            rightZipPos = INIT_RIGHT_POS;
        }

        if (gamepad2.left_bumper) {
            leftZipPos = INIT_LEFT_POS;
        }
        else if (gamepad2.right_bumper) {
            rightZipPos = INIT_RIGHT_POS;
        }
        if (gamepad2.left_trigger > .1) {
            leftZipPos = INIT_LEFT_POS;
            rightZipPos = INIT_RIGHT_POS;
        }

        // Set the positions
        leftZipPos = scaleServo(leftZipPos);
        rightZipPos = scaleServo(rightZipPos);
        leftZip.setPosition(leftZipPos);
        rightZip.setPosition(rightZipPos);

        /*
         * Pullup
         */
        //Do a pullup
        double pup = gamepad2.right_stick_y;
        if(Math.abs(pup) > .5 && pullupEngaged) {
            pullup.setPower(pup);
        } else {
            pullup.setPower(0);
        }

        //manual aim control
        if(hangPos>.95) hangPos=.95;
        if(hangPos<.05) hangPos=.05;
        if (gamepad2.dpad_down) {
            hangPos += AIM_SPEED;
        }
        else if (gamepad2.dpad_up) {
            hangPos -= AIM_SPEED;
        }
        aim.setPosition(hangPos);

        /*
         * Placement
         */
        double cspeed = gamepad2.left_stick_x;
//        cspeed = sign(cspeed) * Math.pow(cspeed, 4);
        cspeed = Math.abs(cspeed) > .2 ? cspeed : 0.0;
        conveyor.setPower(cspeed/4);
        if (gamepad2.right_stick_x > .5 && !pullupEngaged) {
            leftDoor.setPosition(DOOR_UP);
            rightDoor.setPosition(DOOR_DOWN);
        }
        else if (gamepad2.right_stick_x < -.5 && !pullupEngaged) {
            leftDoor.setPosition(DOOR_DOWN);
            rightDoor.setPosition(DOOR_UP);
        }
        else {
            leftDoor.setPosition(DOOR_UP);
            rightDoor.setPosition(DOOR_UP);
        }

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

        /*
         * Telemetry
         */
        if (gyroBeenInit && useGyro)
            telemetry.addData("Gyro", "IN USE");
        else
            telemetry.addData("Gyro", "DEACTIVATED");

        /*
         Here, we want to know our situation whether or not the gyro is being used,
         So we set it to true for the duration of these telemetry readings.
         */
        boolean temp = useGyro;
        useGyro = true;

        telemetry.addData("Pull up?", pullupEngaged ? "Yes" : "No");

        if (!gyroBeenInit)
            telemetry.addData("Situation", "Unknown");
        else if (tipping())
            telemetry.addData("Situation", "Tipping");
        else if (pullingUp())
            telemetry.addData("Situation", "Pulling Up");
        else if (onRamp())
            telemetry.addData("Situation", "On Ramp");
        else if (onFloor())
            telemetry.addData("Situation", "On Floor");
        useGyro = temp;

//        telemetry.addData("Encoder distance", mL1.getCurrentPosition() - encoderPos);
        telemetry.addData("Left Motor Power", mL1.getPower());
        telemetry.addData("Right Motor Power", mR1.getPower());
        telemetry.addData("Heading", orientation);
        telemetry.addData("Pitch", pitch);

    }

    void moveShields(double pos) {
        pos = scaleServo(pos);
        moveShields(pos, 1.0 - pos);
    }

    void moveShields(double l, double r) {
        l = scaleServo(l);
        r = scaleServo(r);
        leftShield.setPosition(l);
        rightShield.setPosition(r);
    }

    boolean onFloor() {
        return useGyro && gyroBeenInit && Math.abs(pitch) < FLOOR_THRESHOLD;
    }
    boolean onRamp() {
        return useGyro && gyroBeenInit && Math.abs(pitch) > RAMP_MIN_ANGLE;
    }

    boolean tipping() {
        return useGyro && gyroBeenInit && !pullupEngaged && Math.abs(pitch) > TIP_MIN_ANGLE;
    }
    boolean pullingUp() {
        return useGyro && gyroBeenInit && pullupEngaged && Math.abs(pitch) > TIP_MIN_ANGLE;
    }

    double scaleServo(double d) {
        if (d > 1.0)
            return 1;
        else if (d < 0.0)
            return 0;
        else
            return d;
    }
    double sign(double d) {
        if (d >= 0) return 1;
        else return -1;
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
            pitch = rollAngle[0];
        }
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
}
