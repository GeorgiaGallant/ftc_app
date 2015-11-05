package com.qualcomm.ftcrobotcontroller.QuantumMechanics;

/**
 * Created by student on 11/3/15.
 */


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoTest extends LinearOpMode{
    Servo servo1;

    @Override
    public void runOpMode() throws InterruptedException {
        servo1 = hardwareMap.servo.get("servozip");
        servo1.setPosition(0);
        sleep(1000);
        servo1.setPosition(.8);
        sleep(1000);
    }
}

