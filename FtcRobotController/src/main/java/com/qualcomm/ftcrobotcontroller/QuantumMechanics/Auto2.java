package com.qualcomm.ftcrobotcontroller.QuantumMechanics;

/**
 * Created by davis on 3/11/16.
 */
public class Auto2 extends Autonomous{
  @Override
  public void runOpMode() throws InterruptedException {
    setup();
    waitForStart();
    driveTicksStraight(100, 1000);
  }
}
