package com.qualcomm.ftcrobotcontroller.QuantumMechanics;

/**
 * Created by davis on 3/11/16.
 */
public class IMUTest extends Autonomous{
  @Override
  public void runOpMode() throws InterruptedException {
    setup();
    waitForStart();
    while (opModeIsActive()) {
      updateHeading();
      telemetry.addData("Heading", orientation);
    }
  }
}
