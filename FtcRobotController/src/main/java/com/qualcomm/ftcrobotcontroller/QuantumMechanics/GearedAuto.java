package com.qualcomm.ftcrobotcontroller.QuantumMechanics;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by davis on 3/18/16.
 */
public class GearedAuto extends Autonomous {
  double speed = .3;
  public void runOpMode() throws InterruptedException {
    setup();
    waitForStart();
    nom.setPower(-1);
    elevator.setPower(1);
    conveyor.setPower(1);
    moveShields(SHIELD_DOWN);
    sleep(500);

    driveTicksStraight(speed, 2000, 1);
    mL1.setDirection(DcMotor.Direction.FORWARD);
    mR1.setDirection(DcMotor.Direction.REVERSE);
    sleep(500);
    driveTicksStraight(speed, 1000, -1);
    mL1.setDirection(DcMotor.Direction.REVERSE);
    mR1.setDirection(DcMotor.Direction.FORWARD);
    sleep(500);
    driveTicksStraight(speed, 3000, 1);
    sleep(1000);

    driveTicksStraight(speed, 2000, 1);
    mL1.setDirection(DcMotor.Direction.FORWARD);
    mR1.setDirection(DcMotor.Direction.REVERSE);
    sleep(500);
    driveTicksStraight(speed, 1000, -1);
    mL1.setDirection(DcMotor.Direction.REVERSE);
    mR1.setDirection(DcMotor.Direction.FORWARD);
    sleep(500);
    driveTicksStraight(speed, 3000, 1);
    sleep(1000);

  }
}
