package com.qualcomm.ftcrobotcontroller.QuantumMechanics;

/**
 * Created by davis on 4/30/16.
 */
public class DefAuto extends Autonomous{
  public void runOpMode() throws InterruptedException {
    setup();
    waitForStart();
    wait(10000);
    drive(1);
    wait(5000);
    drive(0);
  }
}
