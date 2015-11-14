package com.qualcomm.ftcrobotcontroller.QuantumMechanics;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
//TEST//
/**
 * Created by student on 10/13/15.
 */


public class Autonomous extends LinearOpMode {
    Servo climbers;
    Servo pinion;

    DcMotor mL1;
    DcMotor mL2;
 //   DcMotor mL3;

    DcMotor mR1;
    DcMotor mR2;
  //  DcMotor mR3;


    int initPos;


    private void driveTicks(double power, int ticks) throws InterruptedException{
        climbers = hardwareMap.servo.get("climbers");
        pinion = hardwareMap.servo.get("pinion");
        power = Math.abs(power); // Make sure power is positive
        if (ticks < 0) power *= -1;
        ticks = Math.abs(ticks);

        // Get pos
        initPos = mR2.getCurrentPosition();
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
        initPos = mR2.getCurrentPosition();

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
     //   mL3.setPower(rPower);

        mR1.setPower(rPower);
        //   mR2.setPower(rPower);
        mR2.setPower(rPower);
    //    mR3.setPower(rPower);

    }

    private int driveEncoders () {


        return Math.abs(mR2.getCurrentPosition() - initPos) ;
    }


    @Override

    public void runOpMode() throws InterruptedException {
        mL1 = hardwareMap.dcMotor.get("mL1");
        //   mL2 = hardwareMap.dcMotor.get("mL2");
        mL2 = hardwareMap.dcMotor.get("mL2");
     //   mL3 = hardwareMap.dcMotor.get("mL3");

        mR1 = hardwareMap.dcMotor.get("mR1");
        // mR2 = hardwareMap.dcMotor.get("mR2");
        mR2 = hardwareMap.dcMotor.get("mR2");
      //  mR3 = hardwareMap.dcMotor.get("mR3");


        mR1.setDirection(DcMotor.Direction.REVERSE);
        //  mR2.setDirection(DcMotor.Direction.REVERSE);
        mL1.setDirection(DcMotor.Direction.REVERSE);
        mL2.setDirection(DcMotor.Direction.REVERSE);


        waitForStart();
//get to bucket
        driveTicks(-.5, 200);
        sleep(200);


   //     turnTicks(.5, 1600);
    //    sleep(200);


   //     driveTicks(-.5, 2000);
  //      sleep(200);



        //go up
    pinion.setPosition(.35);
        sleep(1000);
        pinion.setPosition(1);
        sleep(1000);

//dumbs climbers
        climbers.setPosition(.1);
        sleep(1000);
        climbers.setPosition(1);
        sleep(1000);

    }
}
