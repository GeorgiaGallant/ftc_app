package com.qualcomm.ftcrobotcontroller.QuantumMechanics;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by davis on 3/18/16.
 */
public abstract class FloorAuto extends Autonomous {
  double speed = .5;
  long DELAY = 1200;
  abstract double dir();
  public void runOpMode() throws InterruptedException {
    setup();
    waitForStart();
    telemetry.addData("Waiting", "...");
    sleep(DELAY);
    telemetry.clearData();
    nom.setPower(-1);
    elevator.setPower(1);
    conveyor.setPower(dir());
    moveShields(SHIELD_DOWN);
    sleep(500);

    driveTicks(speed, 2000);
    mL1.setDirection(DcMotor.Direction.FORWARD);
    mR1.setDirection(DcMotor.Direction.REVERSE);
    sleep(500);
    driveTicks(speed, 1000);
    mL1.setDirection(DcMotor.Direction.REVERSE);
    mR1.setDirection(DcMotor.Direction.FORWARD);
    sleep(500);
    driveTicks(speed, 3000);
    sleep(1000);

    driveTicks(speed, 2000);
    mL1.setDirection(DcMotor.Direction.FORWARD);
    mR1.setDirection(DcMotor.Direction.REVERSE);
    sleep(500);
    driveTicks(speed, 1000);
    mL1.setDirection(DcMotor.Direction.REVERSE);
    mR1.setDirection(DcMotor.Direction.FORWARD);
    sleep(500);
    driveTicks(speed, 3000);
    sleep(1000);

  }
}
