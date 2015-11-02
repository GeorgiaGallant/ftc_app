package com.qualcomm.ftcrobotcontroller.QuantumMechanics;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
//TEST//
/**
 * Created by student on 10/13/15.
 */


public class Autonomous extends LinearOpMode {

    DcMotor mL1;
    DcMotor mL2;
    DcMotor mL3;

    DcMotor mR1;
    DcMotor mR2;
    DcMotor mR3;


    int initPos;


    private void driveTicks(double power, int ticks) throws InterruptedException{

        power = Math.abs(power); // Make sure power is positive
        if (ticks < 0) power *= -1;
        ticks = Math.abs(ticks);

        // Get pos
        initPos = mL1.getCurrentPosition();
        // tFR = mFR.getCurrentPosition();

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

    }

    // pos ticks = right turn
    private void turnTicks (double power, int ticks) throws InterruptedException {
        power = Math.abs(power); // Make sure power is positive
        if (ticks < 0) power *= -1;
        ticks = Math.abs(ticks);

        // Get pos
        initPos = mL1.getCurrentPosition();

        drive(power, -power);

        while(driveEncoders() < ticks){ // average of both encoders


            waitOneFullHardwareCycle();

        }
        drive(0); // Stop motors
    }

    private void drive (double power) {
        drive(power, power);
    }

    private void drive (double lPower, double rPower) {
        mL1.setPower(lPower);
        //    mL2.setPower(lPower);
        mL2.setPower(lPower);
        mL3.setPower(rPower);

        mR1.setPower(rPower);
        //   mR2.setPower(rPower);
        mR2.setPower(rPower);
        mR3.setPower(rPower);

    }

    private int driveEncoders () {


        return Math.abs(mL1.getCurrentPosition() - initPos) ;
    }


    @Override

    public void runOpMode() throws InterruptedException {
        mL1 = hardwareMap.dcMotor.get("mL1");
        //   mL2 = hardwareMap.dcMotor.get("mL2");
        mL2 = hardwareMap.dcMotor.get("mL2");
        mL3 = hardwareMap.dcMotor.get("mL3");

        mR1 = hardwareMap.dcMotor.get("mR1");
        // mR2 = hardwareMap.dcMotor.get("mR2");
        mR2 = hardwareMap.dcMotor.get("mR2");
        mR3 = hardwareMap.dcMotor.get("mR3");


        mR1.setDirection(DcMotor.Direction.REVERSE);
        //  mR2.setDirection(DcMotor.Direction.REVERSE);
        mR2.setDirection(DcMotor.Direction.REVERSE);
        mR3.setDirection(DcMotor.Direction.REVERSE);


        waitForStart();

        driveTicks(.5, -2000);
        sleep(200);


        turnTicks(.5, 1600);
        sleep(200);


        driveTicks(.5, -2000);
        sleep(200);


        turnTicks(.5, 1600);
        sleep(200);


        driveTicks(.8, -2000);
        sleep(200);

        driveTicks(.8, 1000);
        sleep(200);
        driveTicks(.8, -1000);
        sleep(200);
        driveTicks(.5, 5000);
        sleep(200);

        turnTicks(.5, 800);
        sleep(200);


        driveTicks(1, - 5000);

    }
}
