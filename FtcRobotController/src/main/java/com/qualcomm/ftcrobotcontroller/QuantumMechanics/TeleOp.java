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
        Servo ziplineL;
        Servo ziplineR;
        Servo pullupS;
        Servo rightDoor;
        Servo leftDoor;

        //values for the pullup
        double hangPos = .1;
        double maxChangeRate = .01;
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

            //getting servos
            pullupS = hardwareMap.servo.get("pullupS");
            rightDoor = hardwareMap.servo.get("rightDoor");
            leftDoor = hardwareMap.servo.get("leftDoor");
            ziplineL = hardwareMap.servo.get("ziplineL");
            ziplineR = hardwareMap.servo.get("ziplineR");

            //setting motor directions
            mR1.setDirection(DcMotor.Direction.FORWARD);
            mL1.setDirection(DcMotor.Direction.FORWARD);
            nom.setDirection(DcMotor.Direction.FORWARD);
            pullup.setDirection(DcMotor.Direction.REVERSE);
            conveyor.setDirection(DcMotor.Direction.FORWARD);

            //set servo positions
            rightDoor.setPosition(0);
            leftDoor.setPosition(0);
            ziplineL.setPosition(0);
            ziplineR.setPosition(0);
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

            //frontback controls the robot's front and back direction
            float frontBack = gamepad1.left_stick_x;
            //leftright controls the robot's left and right direction
            float leftRight = gamepad1.left_stick_y;

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

            //GAMEPAD 2

            //nom control
            if(gamepad2.a) {
                nom.setPower(-.7);
            }
            else{
                nom.setPower(0);
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

            //door positions

            // clip the position values so that they never exceed their allowed range.
            // armPosition = Range.clip(armPosition, ARM_MIN_RANGE, ARM_MAX_RANGE);
            // clawPosition = Range.clip(clawPosition, CLAW_MIN_RANGE, CLAW_MAX_RANGE);

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


