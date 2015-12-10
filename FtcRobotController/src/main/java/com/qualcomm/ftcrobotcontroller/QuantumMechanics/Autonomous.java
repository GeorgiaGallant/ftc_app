package com.qualcomm.ftcrobotcontroller.QuantumMechanics;

import android.hardware.SensorEventListener;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import android.hardware.*;


//TEST//
/**
 * Created by student on 10/13/15.
 */


public class Autonomous extends LinearOpMode implements SensorEventListener {

    Servo climbers;
    Servo pinion;

    DcMotor mL1;
    DcMotor mL2;
 //   DcMotor mL3;

    DcMotor mR1;
    DcMotor mR2;
  //  DcMotor mR3;


    int initPos;

    float orientation = 0;
    private SensorManager mSensorManager;

    @Override
    public void onSensorChanged(SensorEvent event) {

        // get the angle around the z-axis rotated
        float degree = Math.round(event.values[0]);

        orientation = degree;

    }

    @Override


    private void driveTicks(double power, int ticks) throws InterruptedException{
        climbers = hardwareMap.servo.get("climbers");
        pinion = hardwareMap.servo.get("pinion");
        power = Math.abs(power); // Make sure power is positive
        if (ticks < 0) power *= -1;
        ticks = Math.abs(ticks);

        // Get pos
        initPos = mR2.getCurrentPosition();
        // tFR = mFR.getCurrentPosition();

        drive(power); // Start driving

        while(driveEncoders() < ticks){ // average of both encoders


            // If nearing target ticks, slow down
            if((ticks - driveEncoders()) < 200) {
                if (Math.abs(power) > 0.18) {
                    drive((power > 0) ? 0.18 : -0.18);
                } else drive(power);
            }

            waitOneFullHardwareCycle();

        }
        drive(0); // Stop motors

    }

    // pos ticks = right turn
    private void turnTicks (double power, int ticks) throws InterruptedException {
        power = Math.abs(power); // Make sure power is positive
        if (ticks < 0) power *= -1;
        ticks = Math.abs(ticks);

        // Get pos
        initPos = mR2.getCurrentPosition();

        drive(power, -power);

        while(driveEncoders() < ticks){ // average of both encoders


            waitOneFullHardwareCycle();

        }
        drive(0); // Stop motors
    }

    private void drive(double power) {
        drive(power, power);
    }

    private void drive (double lPower, double rPower) {
        mL1.setPower(lPower);
        //    mL2.setPower(lPower);
        mL2.setPower(lPower);
     //   mL3.setPower(rPower);

        mR1.setPower(rPower);
        //   mR2.setPower(rPower);
        mR2.setPower(rPower);
    //    mR3.setPower(rPower);

    }

    private int driveEncoders () {


        return Math.abs(mR2.getCurrentPosition() - initPos) ;
    }

    void initCompassSensor() {
        SensorManager mSensorManager;
        mSensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);


    }

    @Override
    public void runOpMode() throws InterruptedException {
        mL1 = hardwareMap.dcMotor.get("mL1");
        //   mL2 = hardwareMap.dcMotor.get("mL2");
        mL2 = hardwareMap.dcMotor.get("mL2");
     //   mL3 = hardwareMap.dcMotor.get("mL3");

        mR1 = hardwareMap.dcMotor.get("mR1");
        // mR2 = hardwareMap.dcMotor.get("mR2");
        mR2 = hardwareMap.dcMotor.get("mR2");
      //  mR3 = hardwareMap.dcMotor.get("mR3");


        mR1.setDirection(DcMotor.Direction.REVERSE);
        //  mR2.setDirection(DcMotor.Direction.REVERSE);
        mL1.setDirection(DcMotor.Direction.REVERSE);
        mL2.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
//get to bucket
        driveTicks(-.5, 10650);
        wait1Msec(200);



      //turnTicks(.2, -1265);
    //    sleep(200);


     //   driveTicks(-.5, 9155);
      //  sleep(200);



        //go up
    //pinion.setPosition(.35);
        wait1Msec(1000);
        pinion.setPosition(1);
    //dumps climbers
       // climbers.setPosition(.1);
//        wait1Msec(3000);
//        driveTicks(-.5, 250);
//        climbers.setPosition(.8);
//        wait1Msec(1000);
//        climbers.setPosition(.84);
//        wait1Msec(1000);
//        driveTicks(-.5, 100);
//        wait1Msec(500);
//        climbers.setPosition(.8);
//        wait1Msec(500);
//        climbers.setPosition(.84);

    }

    void wait1Msec(long msecs) throws InterruptedException{
        long startTime = System.currentTimeMillis();

        while(startTime + msecs > System.currentTimeMillis()) {
            waitOneFullHardwareCycle();
        }
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        //
        image = (ImageView) findViewById(R.id.main_iv);

        // TextView that will tell the user what degree is he heading
        tvHeading = (TextView) findViewById(R.id.tvHeading);

        // initialize your android device sensor capabilities
        mSensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
    }

    @Override
    protected void onResume() {
        super.onResume();

        // for the system's orientation sensor registered listeners
        mSensorManager.registerListener(this, mSensorManager.getDefaultSensor(Sensor.TYPE_ORIENTATION),
                SensorManager.SENSOR_DELAY_GAME);
    }

    @Override
    protected void onPause() {
        super.onPause();

        // to stop the listener and save battery
        mSensorManager.unregisterListener(this);
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        // not in use
    }



}
