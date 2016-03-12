package com.qualcomm.ftcrobotcontroller.QuantumMechanics;

import android.hardware.SensorManager;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.ftcrobotcontroller.QuantumMechanics.AdafruitIMU;
import com.qualcomm.robotcore.exception.RobotCoreException;


//TEST//
/**
 * Created by student on 10/13/15.
 */


public class Autonomous extends LinearOpMode{
    Servo ziplineL;
    Servo ziplineR;
    Servo pullupS;
    //double doorRV = 0;
    // double doorLV = .7;
    Servo rightDoor;
    Servo leftDoor;
    Servo arm;

    DcMotor nomF;
    boolean armPressed = false;
    boolean armState = false;
    double armPos1 =.385;
    double armPos2 =.9;
    //values for the pullup
    double hangPos = .1;
    double maxChangeRate = .01;

    DcMotor mL1;
    DcMotor mR1;
    int initPos;

    //Nathaniel's Sensor Variables:
    double orientation = 0;
    boolean hasStarted = false;
    double prevHeading = 0;
    long systemTime = System.nanoTime();
    long prevTime = System.nanoTime();
    AdafruitIMU gyro;

    //The following arrays contain both the Euler angles reported by the IMU (indices = 0) AND the
    // Tait-Bryan angles calculated from the 4 components of the quaternion vector (indices = 1)
    static volatile double[] rollAngle = new double[2], pitchAngle = new double[2], yawAngle = new double[2], accs = new double[3];
    public void updateHeading() {
        //Update gyro values
        gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
        prevHeading = orientation;
        orientation = yawAngle[0];

        //Display information on screen
        //telemetry.addData("Headings(yaw): ",
        //        String.format("Euler= %4.5f", yawAngle[0]));

    }
    /**
     * Drive forward until encoder tick threshold is met.
     * @param power
     * @param ticks Number of encoder ticks to travel
     */
    public void driveTicksStraight(double power, int ticks) {
        int start = mR1.getCurrentPosition();

        updateHeading();
        double initHeading = orientation;
        double error_const = .04;

        while (Math.abs(mR1.getCurrentPosition() - start) < ticks) {
            updateHeading();
            //gyro is too finicky to do integral stuff so just the basic derivative stuff
            double pl = power;
            double pr = power;

            double error = orientation - initHeading;

            pl-=error * error_const;
            pr+=error * error_const;

            pl = scale(pl);
            pr = scale(pr);

            drive(pl, pr);
            telemetry.addData("m1:", Math.abs(mR1.getCurrentPosition() - start) - ticks);
        }

        drive(0);
    }

    public void turnToHeading(double turnPower, double desiredHeading) {
        updateHeading();
        if (orientation+180 > desiredHeading+180) {
      /* might need a dead zone for turning... */
            //Turn left until robot reaches the desiredHeading
            while (orientation+180 > desiredHeading+180) {
                updateHeading();
                drive(turnPower, -turnPower);
            }
            drive(0);
        } else {
            //Turn right until robot reaches the desiredHeading
            while (orientation+180 < desiredHeading+180) {
                updateHeading();
                drive(-turnPower, turnPower);
            }
            drive(0);
        }
        updateHeading();
    }
    public void rotateDegs(double turnPower, double degs) {
        updateHeading();
        double h = orientation + degs * -sign(turnPower);
        if (Math.abs(h) > 180)
            h += -360*sign(h);
        turnToHeading(Math.abs(turnPower), h);
    }

    /**
     * Determine if a number is positive or negative
     * @param d number
     * @return 1 if number is positive, -1 if it's negative
     */
    double sign(double d) {
        if (d >= 0) return 1.0;
        else return -1.0;
    }

    double scale(double d) {
        if (d > 1) return 1.0;
        if (d < -1) return -1.0;
        else return d;
    }

    private void driveTicks(double power, int ticks) throws InterruptedException{

        power = Math.abs(power); // Make sure power is positive
        if (ticks < 0) power *= -1;
        ticks = Math.abs(ticks);

        // Get pos
        initPos = mR1.getCurrentPosition();

        drive(power); // Start driving

        while(driveEncoders() < ticks){ // average of both encoders
            // If nearing target ticks, slow down
            if((ticks - driveEncoders()) < 200) {
                if (Math.abs(power) > 0.18) {
                    drive((power > 0) ? 0.18 : -0.18);
                } else drive(power);
            }

            waitOneFullHardwareCycle();
        }
        drive(0); // Stop motors
        sleep(500);

    }

    // pos ticks = right turn
    private void turnTicks (double power, int ticks) throws InterruptedException {
        power = Math.abs(power); // Make sure power is positive
        if (ticks < 0) power *= -1;
        ticks = Math.abs(ticks);

        // Get pos
        initPos = mR1.getCurrentPosition();

        drive(power, -power);

        while(driveEncoders() < ticks){ // average of both encoders


            waitOneFullHardwareCycle();

        }
        drive(0); // Stop motors
        sleep(500);

    }

    private void drive(double power) {
        drive(power, power);
    }

    private void drive (double lPower, double rPower) {
        mL1.setPower(lPower);
        //    mL2.setPower(lPower);
        // mL2.setPower(lPower);
        //   mL3.setPower(rPower);

        mR1.setPower(rPower);
        //   mR2.setPower(rPower);
        // mR2.setPower(rPower);
        //    mR3.setPower(rPower);

    }

    private int driveEncoders () {
        telemetry.addData("ticks R", mR1.getCurrentPosition());
        telemetry.addData("ticks L", mL1.getCurrentPosition());
        return Math.abs(mR1.getCurrentPosition() - initPos);

    }


    public void setup() {
        mL1 = hardwareMap.dcMotor.get("mL1");
        mR1 = hardwareMap.dcMotor.get("mR1");
        mL1.setDirection(DcMotor.Direction.REVERSE);
        pullupS = hardwareMap.servo.get("pullupS");
        rightDoor = hardwareMap.servo.get("rightDoor");
        leftDoor = hardwareMap.servo.get("leftDoor");
        ziplineL = hardwareMap.servo.get("ziplineL");
        ziplineR = hardwareMap.servo.get("ziplineR");
        arm = hardwareMap.servo.get("arm");
        nomF = hardwareMap.dcMotor.get("nomF");

        //set servo positions
        leftDoor.setPosition(.8);
        rightDoor.setPosition(.1);
        ziplineL.setPosition(1);
        ziplineR.setPosition(0);
        pullupS.setPosition(.1);
        arm.setPosition(armPos2);

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

    @Override
    public void runOpMode() throws InterruptedException {
        setup();
        waitForStart();

        sleep(1000);
        nomF.setPower(-1);
        driveTicks(.8, 550);
        turnTicks(.8, 1000);
        driveTicks(.8, 7600);
        turnTicks(.8, -1950);
        nomF.setPower(0);
        driveTicks(.8, -7000);
    }
}
