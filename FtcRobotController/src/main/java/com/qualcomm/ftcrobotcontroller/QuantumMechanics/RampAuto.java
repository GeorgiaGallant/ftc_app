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
    driveTicksStraight(speed, 1500, 1); // drive straight
    sleep(500);
    rotateDegs(1, 36 * dir); // turn right
    updateHeading();
    sleep(500);
    driveTicksStraight(speed, 2500, 1); //go forward
    sleep(500);
    rotateDegs(1, -80 * dir); // turn to face the ramp
    updateHeading();
    mL1.setDirection(DcMotor.Direction.FORWARD);
    mR1.setDirection(DcMotor.Direction.REVERSE);
    sleep(500);
    moveShields(SHIELD_UP); // raise shields
    elevator.setPower(0); // stop the stuff
    conveyor.setPower(0);
    nom.setPower(0);
    sleep(500);
//    driveTicksStraight(.8, 7000, -1); // up the ramp

    int start = mL1.getCurrentPosition();
    updateHeading();
    double initHeading = orientation;
    boolean tipping = false;

    while (!tipping && Math.abs(mL1.getCurrentPosition() - start) < 7000) {
      driveStraight(.8, initHeading, -1);
      if (Math.abs(pitchAngle[0]) > 50) {
        tipping = true;
//        break;
      }
    }
    if (tipping) {
      while (Math.abs(pitchAngle[0]) > 5) {
        mL1.setPower(-1);
        mR1.setPower(-1);
      }

    }

    sleep(5000);
  }
}
