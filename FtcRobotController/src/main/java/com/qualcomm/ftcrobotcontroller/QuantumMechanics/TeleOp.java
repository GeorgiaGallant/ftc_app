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
    //        double ziplineLV = 1;
//        double ziplineRV = 0;
//        Servo ziplineL;
//        Servo ziplineR;
    Servo pullupS;
    double doorRV = 0;
    double doorLV = .7;
    //        Servo rightDoor;
//        Servo leftDoor;
//        Servo hook;
//        Servo arm;
    // boolean hooks = false;
//        boolean armPressed;
//        boolean armState;
//        double armPos1 =0;
//        double armPos2 =1;
    //values for the pullup
    double hangPos = .5;
    double maxChangeRate = .01;

    int encoderPos;

//        double motorChangeRate = .05;

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
        encoderPos = mR1.getCurrentPosition();
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
//            arm = hardwareMap.servo.get("arm");

        //setting motor directions
        mR1.setDirection(DcMotor.Direction.FORWARD);
        mL1.setDirection(DcMotor.Direction.REVERSE);
        nom.setDirection(DcMotor.Direction.REVERSE);
        pullup.setDirection(DcMotor.Direction.REVERSE);
        conveyor.setDirection(DcMotor.Direction.FORWARD);

//            //set servo positions
//            leftDoor.setPosition(.8);
//            rightDoor.setPosition(0);
//            ziplineL.setPosition(1);
//            ziplineR.setPosition(0);
//            hook.setPosition(0);
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
        float leftRight = gamepad1.left_stick_x;
        //leftright controls the robot's left and right direction
        float frontBack = gamepad1.left_stick_y;

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
            mL1.setPower(-1);
            mR1.setPower(-1);
        }
        else if (frontBack < -.2){ //go backward
            mL1.setPower(1);
            mR1.setPower(1);
        }
        else if (leftRight > .2){ //goes left
            mL1.setPower(-1);
            mR1.setPower(1);
        }
        else if (leftRight < -.2){ //goes right
            mL1.setPower(1);
            mR1.setPower(-1);
        }
        else { //keeps robot still
            mL1.setPower(0);
            mR1.setPower(0);
        }

        if(gamepad1.dpad_down){
            mL1.setPower(.3);
            mR1.setPower(.3);
        }
        if(gamepad1.y){
            mL1.setPower(-.45);
            mR1.setPower(-.45);
        }

        if(gamepad1.dpad_up){
            mL1.setPower(-.80);
            mR1.setPower(-.80);
        }


//            if(gamepad1.a){
//                hook.setPosition(.55);
//            }


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
        else if(gamepad2.y) {
            nomF.setPower(-1);
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
        if(hangPos>.95) hangPos=.95;
        if(hangPos<.05) hangPos=.05;
        if (gamepad2.dpad_down) hangPos += maxChangeRate;
        else if (gamepad2.dpad_up) hangPos -= maxChangeRate;
        pullupS.setPosition(hangPos);

        //conveyerbelt control slow
        if (gamepad2.dpad_right){
            conveyor.setPower(.4);
        }
        else if (gamepad2.dpad_left){
            conveyor.setPower(-.4);
        }
        else{
            conveyor.setPower(0);
        }
//            }
//
//            //zipline positions
//
//           //toggle code
////            if(gamepad2.b && ziplineRV==0) ziplineRV = .82;
////            if(gamepad2.b && ziplineRV == .82) ziplineRV = 0;
////            ziplineR.setPosition(ziplineRV);
//            //press code
//            if(gamepad2.b){
//                ziplineR.setPosition(.82);
//            }
//            else{
//                ziplineR.setPosition(0);
//            }
//
//            //toggle code
////            if(gamepad2.x && ziplineLV == 1) ziplineLV = .18;
////            if(gamepad2.x && ziplineLV == .18) ziplineLV = 1;
////            ziplineL.setPosition(ziplineLV);
//            //press code
//            if(gamepad2.x){
//                ziplineL.setPosition(.18);
//            }
//            else{
//                ziplineL.setPosition(1);
//            }



        //door positions
        //toggle
//            if(gamepad1.b && doorRV == 0) doorRV = .5;
//            if(gamepad1.b && doorRV == .5) doorRV = 0;
//            rightDoor.setPosition(doorRV);
//
//            if(gamepad1.x && doorLV == .7) doorLV = 0;
//            if(gamepad1.x && doorLV == 0) doorLV = .7;
//            leftDoor.setPosition(doorLV);

        //press
//            if(gamepad1.b){
//                leftDoor.setPosition(0);
//            }
//            else{
//                leftDoor.setPosition(.7);
//            }
//            if(gamepad1.x){
//                rightDoor.setPosition(.5);
//            }
//            else{
//                rightDoor.setPosition(0);
//            }
//
//            //toggle arm
//            if(gamepad1.y) armPressed = true;
//            else if(armPressed) {
//                armState = !armState;
//                armPressed = false;
//            }
//            if(armState) arm.setPosition(armPos1);
//            else arm.setPosition(armPos2);
		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 *
		 *
		 *
		 */
        telemetry.addData("Encoder distance", mR1.getCurrentPosition() - encoderPos);
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








//package com.qualcomm.ftcrobotcontroller.QuantumMechanics;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//
//
///**
// * Created by student on 1/29/16.
// *
//    /* Copyright (c) 2014 Qualcomm Technologies Inc
//
//All rights reserved.
//
//Redistribution and use in source and binary forms, with or without modification,
//are permitted (subject to the limitations in the disclaimer below) provided that
//the following conditions are met:
//
//Redistributions of source code must retain the above copyright notice, this list
//of conditions and the following disclaimer.
//
//Redistributions in binary form must reproduce the above copyright notice, this
//list of conditions and the following disclaimer in the documentation and/or
//other materials provided with the distribution.
//
//Neither the name of Qualcomm Technologies Inc nor the names of its contributors
//may be used to endorse or promote products derived from this software without
//specific prior written permission.
//
//NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
//LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
//THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
//FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
//DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
//SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
//OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */
//
////    package com.qualcomm.ftcrobotcontroller.QuantumMechanics;
////    /////
////    import com.qualcomm.robotcore.eventloop.opmode.OpMode;
////    import com.qualcomm.robotcore.hardware.DcMotor;
////    import com.qualcomm.robotcore.hardware.Servo;
////    import com.qualcomm.robotcore.util.Range;
//
//
//
//    /**
//     * TeleOp Mode
//     * <p>
//     * Enables control of the robot via the gamepad
//     */
//    public class TeleOp extends OpMode {
//
//        /*
//         * Note: the configuration of the servos is such that
//         * as the arm servo approaches 0, the arm position moves up (away from the floor).
//         * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
//         */
//        // TETRIX VALUES
//        DcMotor mL1;
//        DcMotor mR1;
//     //   DcMotor nom;
//        DcMotor pullup;
//      //  DcMotor conveyor;
//       // DcMotor nomF;
//        double ziplineLV = 1;
//        double ziplineRV = 0;
////        Servo ziplineL;
////        Servo ziplineR;
//        Servo pullupS;
//        //double doorRV = 0;
////       // double doorLV = .7;
////        Servo rightDoor;
////        Servo leftDoor;
//
////        Servo arm;
//
//        boolean armPressed = false;
//        boolean armState = false;
//        double armPos1 =.30;
//        double armPos2 =.9;
//        //values for the pullup
//        double hangPos = .1;
//        double maxChangeRate = .01;
//
////        double motorChangeRate = .05;
//
//        /**
//         * Constructor
//         */
//        public TeleOp() {
//
//        }
//
//        /*
//         * Code to run when the op mode is first enabled goes here
//         *
//         * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
//         */
//        @Override
//        public void init() {
//
//
//		/*
//		 * Use the hardwareMap to get the dc motors and servos by name. Note
//		 * that the names of the devices must match the names used when you
//		 * configured your robot and created the configuration file.
//		 */
//
//		/*
//		 * For the demo Tetrix K9 bot we assume the following,
//		 *   There are two motors "motor_1" and "motor_2"
//		 *   "motor_1" is on the right side of the bot.
//		 *   "motor_2" is on the left side of the bot and reversed.
//		 *
//		 * We also assume that there are two servos "servo_1" and "servo_6"
//		 *    "servo_1" controls the arm joint of the manipulator.     jkhgkgukhuk
//		 *    "servo_6" controls the claw joint of the manipulator.
//		 */
//            //getting motors
//            mL1 = hardwareMap.dcMotor.get("mL1");
//            mR1 = hardwareMap.dcMotor.get("mR1");
////            nom = hardwareMap.dcMotor.get("nom");
//            pullup = hardwareMap.dcMotor.get("pullup");
////            conveyor = hardwareMap.dcMotor.get("conveyor");
////            nomF = hardwareMap.dcMotor.get("nomF");
////
////            //getting servos
//            pullupS = hardwareMap.servo.get("pullupS");
////            rightDoor = hardwareMap.servo.get("rightDoor");
////            leftDoor = hardwareMap.servo.get("leftDoor");
////            ziplineL = hardwareMap.servo.get("ziplineL");
////            ziplineR = hardwareMap.servo.get("ziplineR");
////            arm = hardwareMap.servo.get("arm");
//
//            //setting motor directions
//            mR1.setDirection(DcMotor.Direction.FORWARD);
//            mL1.setDirection(DcMotor.Direction.REVERSE);
////            nom.setDirection(DcMotor.Direction.REVERSE);
//            pullup.setDirection(DcMotor.Direction.REVERSE);
////            conveyor.setDirection(DcMotor.Direction.FORWARD);
////
////            //set servo positions
////            leftDoor.setPosition(.8);
////            rightDoor.setPosition(.1);
////            ziplineL.setPosition(1);
////            ziplineR.setPosition(0);
//
//
//            pullupS.setPosition(.5);
////            arm.setPosition(armPos2);
//
//        }
//        /*
//         * This method will be called repeatedly in a loop
//         *
//         * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
//         *
//         *
//         */
//
//        @Override
//        public void loop() {
//		/*
//		 * Gamepad 1
//		 *
//		 * Gamepad 1 controls the motors via the left stick, and it controls the
//		 * wrist/claw via the a,b, x, y buttons
//		 */
//
//
//            //naive driving
//
//            //frontback controls the robot's front and back direction
//            float leftRight = gamepad1.right_stick_x;
//            //leftright controls the robot's left and right direction
//            float frontBack = gamepad1.left_stick_y;
//
//            //make sure the motors don't stall!
//            if(leftRight<.2 && leftRight>-.2){
//                leftRight = 0;
//            }
//            if(frontBack<.2 && frontBack>-.2){
//                frontBack = 0;
//            }
//
//            // We aren't scaling motor values to decrease the chance of stalling the motors
//            //double frontBack is the y-axis values of the left stick to drive forwards and backwards
//            //double leftRight takes the x-axis values of the right stick to turn
//            //gamepad 1
//            if (frontBack >.2){ //go forward
//                mL1.setPower(-1);
//                mR1.setPower(-1);
//            }
//            else if (frontBack < -.2){ //go backward
//                mL1.setPower(1);
//                mR1.setPower(1);
//            }
//            else if (leftRight > .2){ //goes left
//                mL1.setPower(-1);
//                mR1.setPower(1);
//            }
//            else if (leftRight < -.2){ //goes right
//                mL1.setPower(1);
//                mR1.setPower(-1);
//            }
//            else { //keeps robot still
//                mL1.setPower(0);
//                mR1.setPower(0);
//            }
//
//            if(gamepad1.dpad_down){
//                mL1.setPower(.3);
//                mR1.setPower(.3);
//            }
//            if(gamepad1.y){
//                mL1.setPower(-.45);
//                mR1.setPower(-.45);
//            }
//
//            if(gamepad1.dpad_up){
//                mL1.setPower(-.80);
//                mR1.setPower(-.80);
//            }
//
//
//
//            //arc turning
//            //the idea behind this is so that we can do gradual turns as opposed to only ninety degree turns
//            //left right
////            float LRr = gamepad1.right_stick_y;
////            if(LRr > .5) {
////                mL1.setPower(1);
////                mR1.setPower(1);
////                mR1.setPower(mR1.getPower() - motorChangeRate);
////                if(mR1.getPower() < .7) mR1.setPower(.7);
////            }
////            if(LRr < -.5) {
////                mR1.setPower(1);
////                mL1.setPower(1);
////                mL1.setPower(mL1.getPower() - motorChangeRate);
////                if(mR1.getPower() < .7) mR1.setPower(.7);
////            }
//
//            //GAMEPAD 2
//
//            //nom control
//
//
////            if(gamepad2.a) {
////                nom.setPower(1);
////                nomF.setPower(1);
////            }
////            else if(gamepad2.y) {
////                nomF.setPower(-1);
////            }
////            else{
////                nom.setPower(0);
////                nomF.setPower(0);
////            }
////
//            //Do a pullup
//            if(gamepad2.right_bumper) {
//                pullup.setPower(-.7);
//            }
//            else if(gamepad2.left_bumper) {
//                pullup.setPower(.7);
//            }
//            else{
//                pullup.setPower(0);
//            }
////
////
////            //Using manual controls to position the tapemeasure during pullup until set positions are finalized
////            //Increases servo position by small increments so that loop makes servo move smoothly
////            //manual pullupS control
//            if(hangPos>.8) hangPos=.8;
//            if(hangPos<.15) hangPos=.15;
//            if (gamepad2.dpad_down) hangPos += maxChangeRate;
//            else if (gamepad2.dpad_up) hangPos -= maxChangeRate;
//            pullupS.setPosition(hangPos);
////
////            //conveyerbelt control slow
////            if (gamepad2.dpad_right){
////                conveyor.setPower(.2);
////            }
////            else if (gamepad2.dpad_left){
////                conveyor.setPower(-.2);
////            }
////            else{
////                conveyor.setPower(0);
////            }
////
////            //zipline positions
////
////           //toggle code
//////            if(gamepad2.b && ziplineRV==0) ziplineRV = .82;
//////            if(gamepad2.b && ziplineRV == .82) ziplineRV = 0;
//////            ziplineR.setPosition(ziplineRV);
////            //press code
////            if(gamepad2.b){
////                ziplineR.setPosition(.82);
////            }
////            else{
////                ziplineR.setPosition(0);
////            }
////
////            //toggle code
//////            if(gamepad2.x && ziplineLV == 1) ziplineLV = .18;
//////            if(gamepad2.x && ziplineLV == .18) ziplineLV = 1;
//////            ziplineL.setPosition(ziplineLV);
////            //press code
////            if(gamepad2.x){
////                ziplineL.setPosition(.11);
////            }
////            else{
////                ziplineL.setPosition(1);
////            }
////
////            //doors
////            if(gamepad1.b){
////                leftDoor.setPosition(0);
////            }
////            else{
////                leftDoor.setPosition(.7);
////            }
////            if(gamepad1.x){
////                rightDoor.setPosition(.7);
////            }
////            else{
////                rightDoor.setPosition(.1);
////            }
////
////
////            //Switches state once "a" button is released
////            //toggle arm to hook onto pull-up bar to stop sliding
////            if(gamepad1.right_bumper) armPressed = true;
////            else if(armPressed) {
////                armState = !armState;
////                armPressed = false;
////            }
////            if(armState) arm.setPosition(armPos1);
////            else arm.setPosition(armPos2);
////
//
//		/*
//		 * Send telemetry data back to driver station. Note that if we are using
//		 * a legacy NXT-compatible motor controller, then the getPower() method
//		 * will return a null value. The legacy NXT-compatible motor controllers
//		 * are currently write only.
//		 *
//		 *
//		 *
//		 */
////        telemetry.addData("Text", "*** Robot Data***");
////            telemetry.addData("arm", "arm:  " + String.format("%.2f", armPosition));
////        telemetry.addData("spool", "spool:  " + String.format("%.2f", spoolPosition));
////
////        //   telemetry.addData("claw", "claw:  " + String.format("%.2f", clawPosition));
////        telemetry.addData("left tgt pwr",  "left  pwr: " + String.format("%.2f", left));
////        telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", right));
//            //    telemetry.addData("left trigger", "left trigger pwr: " + String.format("%.2f", throttle));
//
//        }
//
//        /*
//         * Code to run when the op mode is first disabled goes here
//         *
//         * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
//         */
//        @Override
//        public void stop() {
//
//        }
//    }
//
//
