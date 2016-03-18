package com.qualcomm.ftcrobotcontroller.QuantumMechanics;

/**
 * Created by davis on 3/17/16.
 */
public class DIMTest extends Autonomous{

  public void runOpMode() throws InterruptedException{
    waitForStart();
    while (opModeIsActive()) {
      updateHeading();
      telemetry.addData("Heading", orientation);
      telemetry.addData("Pitch", pitchAngle[0]);
      telemetry.addData("Roll", rollAngle[0]);
    }
  }
}
