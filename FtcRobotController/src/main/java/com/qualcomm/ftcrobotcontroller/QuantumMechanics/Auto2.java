package com.qualcomm.ftcrobotcontroller.QuantumMechanics;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by davis on 3/11/16.
 */
public class Auto2 extends Autonomous{
  @Override
  public void runOpMode() throws InterruptedException {
    setup();
    waitForStart();
    nom.setPower(-1);
    moveShields(SHIELD_DOWN);
    sleep(500);
    driveTicksStraight(.5, 4500, 1);
    sleep(1000);
    rotateDegs(1, -41);
    updateHeading();
    mL1.setDirection(DcMotor.Direction.FORWARD);
    mR1.setDirection(DcMotor.Direction.REVERSE);
    sleep(500);
    nom.setPower(0);
    moveShields(SHIELD_UP);
    sleep(500);
    driveTicksStraight(.5, 5000, -1);
    sleep(1000);
    driveTicksStraight(.8, 3000, -1);
  }
}
