package com.qualcomm.ftcrobotcontroller.QuantumMechanics;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by davis on 3/18/16.
 */
public abstract class RampAuto extends Autonomous{

  abstract double getDir();

  public void runOpMode() throws InterruptedException {
    double dir = getDir();
    double speed = .3;
    setup();
    waitForStart();
//    telemetry.addData("Waiting", "...");
//    sleep(12000);
//    telemetry.clearData();
    nom.setPower(-1);
    elevator.setPower(1);
    conveyor.setPower(1);
    moveShields(SHIELD_DOWN);
    sleep(500);
    driveTicksStraight(speed, 1500, 1);
    sleep(500);
    rotateDegs(1, 36*dir);
    updateHeading();
    sleep(500);
    driveTicksStraight(speed, 2500, 1);
    sleep(500);
    rotateDegs(1, -80*dir);
    updateHeading();
    mL1.setDirection(DcMotor.Direction.FORWARD);
    mR1.setDirection(DcMotor.Direction.REVERSE);
    sleep(500);
    moveShields(SHIELD_UP);
    elevator.setPower(0);
    conveyor.setPower(0);
    nom.setPower(0);
    sleep(500);
    driveTicksStraight(.8, 7000, -1);
    sleep(5000);
  }
}
