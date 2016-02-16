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
        // TETRIX VALUES
        DcMotor mL1;
        DcMotor mR1;
        DcMotor nom;
        DcMotor pullup;
        DcMotor conveyor;
        DcMotor nomF;
//        Servo ziplineL;
//        Servo ziplineR;
        Servo pullupS;
//        Servo rightDoor;
//        Servo leftDoor;

        //values for the pullup
        double hangPos = .1;
        double maxChangeRate = .01;
//        double motorChangeRate = .05;
        boolean door = false;
        boolean zipR = false;
        boolean zipL = false;
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
            //getting motors
            mL1 = hardwareMap.dcMotor.get("mL1");
            mR1 = hardwareMap.dcMotor.get("mR1");
            nom = hardwareMap.dcMotor.get("nom");
            pullup = hardwareMap.dcMotor.get("pullup");
            conveyor = hardwareMap.dcMotor.get("conveyor");
            nomF = hardwareMap.dcMotor.get("nomF");

            //getting servos
            pullupS = hardwareMap.servo.get("pullupS");
//            rightDoor = hardwareMap.servo.get("rightDoor");
//            leftDoor = hardwareMap.servo.get("leftDoor");
//            ziplineL = hardwareMap.servo.get("ziplineL");
//            ziplineR = hardwareMap.servo.get("ziplineR");

            //setting motor directions
            mR1.setDirection(DcMotor.Direction.FORWARD);
            mL1.setDirection(DcMotor.Direction.FORWARD);
            nom.setDirection(DcMotor.Direction.REVERSE);
            pullup.setDirection(DcMotor.Direction.REVERSE);
            conveyor.setDirection(DcMotor.Direction.FORWARD);

            //set servo positions
//            rightDoor.setPosition(0);
//            leftDoor.setPosition(0);
//            ziplineL.setPosition(0);
//            ziplineR.setPosition(0);
        }
        /*
         * This method will be called repeatedly in a loop
         *
         * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
         *
         *
         */

        @Override
        public void loop() {
		/*
		 * Gamepad 1
		 *
		 * Gamepad 1 controls the motors via the left stick, and it controls the
		 * wrist/claw via the a,b, x, y buttons
		 */


            //naive driving

            //frontback controls the robot's front and back direction
            float frontBack = gamepad1.left_stick_x;
            //leftright controls the robot's left and right direction
            float leftRight = gamepad1.right_stick_y;

            //make sure the motors don't stall!
            if(leftRight<.2 && leftRight>-.2){
                leftRight = 0;
            }
            if(frontBack<.2 && frontBack>-.2){
                frontBack = 0;
            }

            // write the values to the motors
            //gamepad 1
            if (frontBack >.2){ //go forward
                mL1.setPower(frontBack);
                mR1.setPower(frontBack);
            }
            else if (frontBack < -.2){ //go backward
                mL1.setPower(frontBack);
                mR1.setPower(frontBack);
            }
            else if (leftRight > .2){ //goes left
                mL1.setPower(leftRight);
                mR1.setPower(-leftRight);
            }
            else if (leftRight < -.2){ //goes right
                mL1.setPower(leftRight);
                mR1.setPower(-leftRight);
            }
            else { //keeps robot still
                mL1.setPower(0);
                mR1.setPower(0);
            }

            //arc turning
            //the idea behind this is so that we can do gradual turns as opposed to only ninety degree turns
            //left right
//            float LRr = gamepad1.right_stick_y;
//            if(LRr > .5) {
//                mL1.setPower(1);
//                mR1.setPower(1);
//                mR1.setPower(mR1.getPower() - motorChangeRate);
//                if(mR1.getPower() < .7) mR1.setPower(.7);
//            }
//            if(LRr < -.5) {
//                mR1.setPower(1);
//                mL1.setPower(1);
//                mL1.setPower(mL1.getPower() - motorChangeRate);
//                if(mR1.getPower() < .7) mR1.setPower(.7);
//            }

            //GAMEPAD 2

            //nom control
            if(gamepad2.a) {
                nom.setPower(1);
                nomF.setPower(1);
            }
            else{
                nom.setPower(0);
                nomF.setPower(0);
            }

            //Do a pullup
            if(gamepad2.right_bumper) {
                pullup.setPower(-.7);
            }
            else if(gamepad2.left_bumper) {
                pullup.setPower(.7);
            }
            else{
                pullup.setPower(0);
            }

            //manual pullupS control
            if(hangPos>1) hangPos=1;
            if(hangPos<.01) hangPos=.01;
            if (gamepad2.dpad_down) hangPos += maxChangeRate;
            else if (gamepad2.dpad_up) hangPos -= maxChangeRate;
            pullupS.setPosition(hangPos);

            //conveyerbelt control
            if (gamepad2.dpad_right){
                conveyor.setPower(.8);
            }
            else if (gamepad2.dpad_left){
                conveyor.setPower(-.8);
            }
            else{
                conveyor.setPower(0);
            }

            //zipline positions
            //right
//            if(gamepad2.b && ziplineR.getPosition()==0) zipR = true;
//            if(gamepad2.b && ziplineR.getPosition()==1) zipR = false;
//            if(zipR) ziplineR.setPosition(1);
//            else ziplineR.setPosition(0);
//            //left
//            if(gamepad2.x && ziplineL.getPosition()==0) zipL = true;
//            if(gamepad2.x && ziplineL.getPosition()==1) zipL = false;
//            if(zipL) ziplineL.setPosition(1);
//            else ziplineL.setPosition(0);
//
//            //door positions
//            if(gamepad2.y) door = !door;
//            if(gamepad2.y && rightDoor.getPosition()==0 && leftDoor.getPosition()==0) door = true;
//            if(gamepad2.y && rightDoor.getPosition()==.5 && leftDoor.getPosition()==.5) door = false;
//            //right
//            if(door) rightDoor.setPosition(.5);
//            else rightDoor.setPosition(0);
//            //left
//            if(door) leftDoor.setPosition(.5);
//            else leftDoor.setPosition(0);

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
    }


