package com.qualcomm.ftcrobotcontroller.QuantumMechanics;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by student on 1/29/16.
 *
    /* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

//    package com.qualcomm.ftcrobotcontroller.QuantumMechanics;
//    /////
//    import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//    import com.qualcomm.robotcore.hardware.DcMotor;
//    import com.qualcomm.robotcore.hardware.Servo;
//    import com.qualcomm.robotcore.util.Range;



    /**
     * TeleOp Mode
     * <p>
     * Enables control of the robot via the gamepad
     */
    public class TeleOp extends OpMode {

        /*
         * Note: the configuration of the servos is such that
         * as the arm servo approaches 0, the arm position moves up (away from the floor).
         * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
         */
        // TETRIX VALUES.
        final static double ARM_MIN_RANGE  = 0.20;
        final static double ARM_MAX_RANGE  = 0.90;
        final static double CLAW_MIN_RANGE  = 0.20;
        final static double CLAW_MAX_RANGE  = 0.7;


        // position of the arm servo.
//    double armPosition;
        double spoolPosition;
        double climbersPosition;

        // amount to change the arm servo position.
        double armDelta = 0.1;

        // position of the claw servo
        double clawPosition;

        // amount to change the claw servo position by
        double clawDelta = 0.1;

        DcMotor mL1;
        DcMotor mR1;
        DcMotor nom;
        DcMotor pullup;
        DcMotor conveyor;

        //     Servo zipline;
        Servo pullupS;
//    Servo rightDoor;
//    Servo leftDoor;


////    Servo arm;
//    Servo climbers;
//    Servo pinion;



        /**
         * Constructor
         */
        public TeleOp() {

        }

        /*
         * Code to run when the op mode is first enabled goes here
         *
         * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
         */
        @Override
        public void init() {


		/*
		 * Use the hardwareMap to get the dc motors and servos by name. Note
		 * that the names of the devices must match the names used when you
		 * configured your robot and created the configuration file.
		 */

		/*
		 * For the demo Tetrix K9 bot we assume the following,
		 *   There are two motors "motor_1" and "motor_2"
		 *   "motor_1" is on the right side of the bot.
		 *   "motor_2" is on the left side of the bot and reversed.
		 *
		 * We also assume that there are two servos "servo_1" and "servo_6"
		 *    "servo_1" controls the arm joint of the manipulator.     jkhgkgukhuk
		 *    "servo_6" controls the claw joint of the manipulator.
		 */

            mL1 = hardwareMap.dcMotor.get("mL1");
            mR1 = hardwareMap.dcMotor.get("mR1");

//    //    elevator = hardwareMap.dcMotor.get("elevator");
            nom = hardwareMap.dcMotor.get("nom");
            pullup = hardwareMap.dcMotor.get("pullup");
            conveyor = hardwareMap.dcMotor.get("conveyor");
            pullupS = hardwareMap.servo.get("pullupS");
//
//
            mR1.setDirection(DcMotor.Direction.FORWARD);
            mL1.setDirection(DcMotor.Direction.FORWARD);
            nom.setDirection(DcMotor.Direction.FORWARD);
            pullup.setDirection(DcMotor.Direction.REVERSE);
            conveyor.setDirection(DcMotor.Direction.FORWARD);



//        //Servo servo1;
//       // servo1 = hardwareMap.servo.get("servozip");
//        double servo1Position;
//        	arm = hardwareMap.servo.get("servozip");
//            spool = hardwareMap.servo.get("spool");
//            climbers = hardwareMap.servo.get("climbers");
//            pinion = hardwareMap.servo.get("pinion");
            //   zipline = hardwareMap.servo.get("zipline");
//


            //	claw = hardwareMap.servo.get("servo_6");

            // assign the starting position of the wrist and claw
//        arm.setPosition(0);
//        spool.setPosition(1);
//        climbers.setPosition(0);
//        pinion.setPosition(.493);

//        zipline.setPosition(.493);


            // servo1.setPosition(0);
            // servo1.setPosition(.8);


        }
        //got some booleans that may or may not be used
//    boolean armOut=false;
//    boolean armPressed=false;
//    boolean armHold = false;

        /*
         * This method will be called repeatedly in a loop
         *
         * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
         *
         *
         */


        double hangPos = .5;
        double maxChangeRate = .01;

        @Override
        public void loop() {

		/*
		 * Gamepad 1
		 *
		 * Gamepad 1 controls the motors via the left stick, and it controls the
		 * wrist/claw via the a,b, x, y buttons
		 */

            // throttle: left_stick_y ranges from -1 to 1, where -1 is full up, and
            // 1 is full down
            // direction: left_stick_x ranges from -1 to 1, where -1 is full left
            // and 1 is full right
            float frontBack = gamepad1.left_stick_x;
            float leftRight = gamepad1.left_stick_y;
            //float right =
            //float left =

            if(leftRight<.2 && leftRight>-.2){
                leftRight = 0;
            }
            if(frontBack<.2 && frontBack>-.2){
                frontBack = 0;
            }
//
//        // clip the right/left values so that the values never exceed +/- 1
//            leftRight = Range.clip(leftRight, -1, 1);
//            frontBack = Range.clip(frontBack, -1, 1);
//


//        // scale the joystick value to make it easier to control
//        // the robot more precisely at slower speeds.
//            right = (float)scaleInput(right);
//            left =  (float)scaleInput(left);
//
//
//        // write the values to the motors
            //gamepad 1
            if (frontBack >.2){ //go forward
                mL1.setPower(frontBack);
                mR1.setPower(frontBack);
            }
            else if (frontBack < -.2){ //go backward
                mL1.setPower(frontBack);
                mR1.setPower(frontBack);
            }
            else if (leftRight > .2){
                mL1.setPower(leftRight);
                mR1.setPower(-leftRight);
            }
            else if (leftRight < -.2){
                mL1.setPower(leftRight);
                mR1.setPower(-leftRight);
            }
            else {
                mL1.setPower(0);
                mR1.setPower(0);
            }

            //nom stuff

            if(gamepad2.a) {
                nom.setPower(-.7);
            }
            else{
                nom.setPower(0);
            }



            //GAMEPAD 2
            //Zipclimber arm
            //when button(a) is pushed, toggle arm
            //test position values!
       /* if(gamepad2.a) armPressed = true;
        if(gamepad2.a==false && armPressed==true) {
            armPressed = false;
            armOut = true;
            if(armOut==true) arm.setPosition(0.92);
            else arm.setPosition(0.0);
        } */

            //Trying to do pullup

            if(gamepad2.right_bumper) {
                pullup.setPower(-.7);
            }
            else if(gamepad2.left_bumper) {
                pullup.setPower(.7);
            }
            else{
                pullup.setPower(0);
            }

            //pullup Servo
//            if (gamepad2.b){
//                pullupS.setPosition(.5);
//            }
//            if (gamepad2.x){
//                pullupS.setPosition(-.5);
//            }
//            //pullup manual
//            double val = pullupS.getPosition();
//            if(gamepad2.dpad_up && val <1) {
//                    pullupS.setPosition(val + .1);
//            }
//            if(gamepad2.dpad_down && val >-1) {
//                    pullupS.setPosition(val - .1);
//            }

            if(hangPos>.9) hangPos=.9;
            if(hangPos<-.9) hangPos=-.9;
            if (gamepad2.dpad_up) hangPos += maxChangeRate;
            else if (gamepad2.dpad_down) hangPos -= maxChangeRate;
            pullupS.setPosition(hangPos);

            //conveyerbelt
            if (gamepad2.dpad_right){
                conveyor.setPower(.8);
            }
            else if (gamepad2.dpad_left){
                conveyor.setPower(-.8);
            }
            else{
                conveyor.setPower(0);
            }

//        //Door
//
//        if (gamepad2.b){
//            rightDoor.setPosition(.);
//        }
//


//        if(gamepad1.right_trigger > 0){
//            zipline.setPosition(1);
//        }
//       if(gamepad2.left_bumper) {
//            arm.setPosition(0.6);
//           telemetry.addData("button a pressed", " ");
//        }
//        else {
//           arm.setPosition(0);
//       }
//        if(gamepad2.right_bumper) {
//            spool.setPosition(0.0);
//        }
//        else {
//            spool.setPosition(1);
//        }
//
////        if(gamepad2.a) {
////            climbers.setPosition(1);
////            telemetry.addData("button a pressed", " ");
////        }
////        else {
////            climbers.setPosition(.1);
////        }
////        if(gamepad2.b){
////            pinion.setPosition(1);
////        }
////        else{
////            pinion.setPosition(.35);
////        }
//
//        //westley code
//        if(gamepad2.dpad_up){
//            tubeLift.setDirection(Servo.Direction.FORWARD);
//            tubeLift.setPosition(.7);
//            telemetry.addData("PRESSED!!!!", " ");
//        }
//        else if(gamepad2.dpad_down){
//            tubeLift.setDirection(Servo.Direction.REVERSE);
//            tubeLift.setPosition(.7);
//        }
//        else{
//            tubeLift.setPosition(.493);
//        }
//
//        if(gamepad2.dpad_left){
//            tubeRotate.setDirection(Servo.Direction.FORWARD);
//            telemetry.addData("left", 0);
//            tubeRotate.setPosition(.55);
//        }
//        else if(gamepad2.dpad_right){
//            tubeRotate.setDirection(Servo.Direction.REVERSE);
//            telemetry.addData("right", 0);
//            tubeRotate.setPosition(.55);
//        }
//        else if(gamepad2.a){
//            telemetry.addData("freeze", 0);
//            tubeRotate.setDirection(Servo.Direction.REVERSE);
//            tubeRotate.setPosition(.75);
//            tubeRotate.setDirection(Servo.Direction.FORWARD);
//            tubeRotate.setPosition(.75);
//            tubeRotate.setPosition(.493);
//
//        }
//        else{
//            tubeRotate.setPosition(.493);
//        }
//
//        if(gamepad2.left_stick_y < 0){
//            tubeAngle.setDirection(Servo.Direction.REVERSE);
//            tubeAngle.setPosition(.55);
//        }
//        else if(gamepad2.left_stick_y > 0){
//            tubeAngle.setDirection(Servo.Direction.FORWARD);
//            tubeAngle.setPosition(.55);
//        }
//        else{
//            tubeAngle.setPosition(.493);
//        }
//




//        if(gamepad2.a) armHold = true;
//        if(armHold==true) arm.setPosition(0.92);
//        if(armHold==false) arm.setPosition(0.0);












            // update the position of the arm

            // clip the position values so that they never exceed their allowed range.
//        armPosition = Range.clip(armPosition, ARM_MIN_RANGE, ARM_MAX_RANGE);
//        clawPosition = Range.clip(clawPosition, CLAW_MIN_RANGE, CLAW_MAX_RANGE);

            // write position values to the wrist and claw servo
            //arm.setPosition(armPosition);
//		claw.setPosition(clawPosition);



		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 *
		 *
		 * Kyle's stuff with triggers is on the Bottom
		 */
//        telemetry.addData("Text", "*** Robot Data***");
//            telemetry.addData("arm", "arm:  " + String.format("%.2f", armPosition));
//        telemetry.addData("spool", "spool:  " + String.format("%.2f", spoolPosition));
//
//        //   telemetry.addData("claw", "claw:  " + String.format("%.2f", clawPosition));
//        telemetry.addData("left tgt pwr",  "left  pwr: " + String.format("%.2f", left));
//        telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", right));
            //    telemetry.addData("left trigger", "left trigger pwr: " + String.format("%.2f", throttle));

        }

        /*
         * Code to run when the op mode is first disabled goes here
         *
         * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
         */
        @Override
        public void stop() {

        }


        /*
         * This method scales the joystick input so for low joystick values, the
         * scaled value is less than linear.  This is to make it easier to drive
         * the robot more precisely at slower speeds.
         */
        double scaleInput(double dVal)  {
            double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                    0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 }; //took out 0.05, 0.09, 0.10

            // get the corresponding index for the scaleInput array.
            int index = (int) (dVal * 16.0);

            // index should be positive.
            if (index < 0) {
                index = -index;
            }

            // index cannot exceed size of array minus 1.
            if (index > 16) {
                index = 16;
            }

            // get value from the array.
            double dScale = 0.0;
            if (dVal < 0) {
                dScale = -scaleArray[index];
            } else {
                dScale = scaleArray[index];
            }

            // return scaled value.
            return dScale;
        }

    }


