package com.qualcomm.ftcrobotcontroller.QuantumMechanics;

import android.util.Log;

import com.qualcomm.robotcore.exception.RobotCoreException;

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
      telemetry.addData("Pitch", pitchAngle[0]);
      telemetry.addData("Roll", rollAngle[0]);
//      telemetry.addData("Roll2", rollAngle[1]);
    }
  }
}
