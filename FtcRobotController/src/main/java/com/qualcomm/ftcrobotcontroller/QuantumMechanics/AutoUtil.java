package com.qualcomm.ftcrobotcontroller.QuantumMechanics;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * Created by davis on 4/28/16.
 */
public abstract class AutoUtil extends LinearOpMode {
  static final double P = .2;

  DcMotor encoder1;
  DcMotor encoder2;

  VoltageSensor volt;

  ColorSensor cs1;
  ColorSensor cs2;

  /* Gyro Methods */

  /**
   * Get the current heading of the robot in degrees.
   * Left of 0 is positive up to 180, right of 0 is negative to -180.
   * @return heading of the robot
   */
  abstract double getHeading();

  /**
   * Reset the offsets on the gyro so that
   * the orientation is now 0 in pitch, roll, and yaw.
   */
  abstract void resetGyro();

  abstract void drive(double l, double r);
  abstract void drive(double pow);

  /**
   * Determine if a number is positive or negative
   * @param d number
   * @return 1 if number is positive, -1 if it's negative
   */
  double sign(double d) {
    if (d >= 0) return 1.0;
    else return -1.0;
  }

  /**
   * Scale a double to be in range for motor power.
   * @param d
   * @return
   */
  double scale(double d) {
    if (d > 1.0)
      return 1.0;
    else if (d < -1.0)
      return -1.0;
    else
      return d;
  }

  public void driveStraight(double power, double targetHeading, int s) {
    //gyro is too finicky to do integral stuff so just the basic derivative stuff
    double pl = power;
    double pr = power;

    double error = getHeading() - targetHeading;

    pl-=error * P*s;
    pr+=error * P*s;

    pl = scale(pl);
    pr = scale(pr);

    drive(pl, pr);
  }

  /**
   * Drive forward until encoder tick threshold is met.
   *
   * TODO: make this work for forward and backwards
   *
   * @param power motor power
   * @param ticks Number of encoder ticks to travel
   */
  public void driveTicksStraight(double power, int ticks, int s) {
    int start = encoder1.getCurrentPosition();
    power = scale(power);
    double initHeading = getHeading();

    while (Math.abs(encoder1.getCurrentPosition() - start) < ticks) {
      driveStraight(power, initHeading, s);
      telemetry.addData("m1:", Math.abs(encoder1.getCurrentPosition() - start));
    }

    drive(0);
  }

  /**
   * Turn to a desired heading
   * @param turnPower
   * @param desiredHeading
   */
  public void turnToHeading(double turnPower, double desiredHeading) {
    // desired heading is more negative means turn counterclockwise
    if (getHeading() > desiredHeading)
      while (getHeading() > desiredHeading)
        drive(-turnPower, turnPower);
    else
      while (getHeading() < desiredHeading)
        drive(turnPower, -turnPower);

    drive(0);
  }

  /**
   * Turn a specific number of degrees, between 180 and -180
   * positive is counter, negative is counterclockwise.
   * @param turnPower motor power for the turn
   * @param degs number of degrees to turn.
   */
  public void rotateDegs(double turnPower, double degs) {
    resetGyro(); // Now forward is 0.
    double h = getHeading() + degs;
    turnToHeading(Math.abs(turnPower), h);
  }
}
