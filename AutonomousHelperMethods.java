double getDistance() {
        distReadings[numDistReadings % distReadings.length] = dist.getValue() * 5.0 / 1024.0; // convert to meters

        numDistReadings++;

        if(numDistReadings < distReadings.length) {
            return Double.MAX_VALUE;
        } else {
            double sum = 0.0;

            for(int i = 0; i < distReadings.length; i++) {
                sum += distReadings[i];
            }

            telemetry.addData("total", sum);
            telemetry.addData("numreadings", numDistReadings);

            return sum / (double) distReadings.length;
        }
    }

    void turnDegrees(double degrees, double motorPower) {
        double current_heading = 0;

        double degreeThreshold = 4;

        try {
            current_heading = angle360(angles.heading);
        } catch (Exception e) {
            e.printStackTrace();
        }

        double initHeading = current_heading;

        double goalHeading = (current_heading + degrees + 360) % 360;

        if (degrees < 0) {
            pl = -motorPower;
            pr = motorPower;
        } else {
            pl = motorPower;
            pr = -motorPower;
        }

        leftMotor.setPower(pl);
        rightMotor.setPower(pr);

        while(Math.abs(goalHeading - current_heading) > degreeThreshold) {
            try {
                current_heading = angles.heading;
            } catch(Exception e) {
                e.printStackTrace();
            }
            telemetry.addData("current_heading:", current_heading);
            telemetry.update();
        }



        pl = pr = 0;
        leftMotor.setPower(pl);
        rightMotor.setPower(pr);

    }



    void driveUntilDistEqualsStraight(double motorPower, double targetDistance) {
        while(getDistance() == Double.MAX_VALUE); //wait until the sensor takes at least 50 readings

        double initial_heading = 0;


        try {
            initial_heading = angles.heading;
        } catch(Exception e){}


        double distanceThreshold = .02;

        while (Math.abs(getDistance() - targetDistance) > distanceThreshold) {

            if (targetDistance < getDistance()) {
                motorPower = -Math.abs(motorPower); //drive backwards!
            } else if (targetDistance > getDistance()) {
                motorPower = Math.abs(motorPower); //drive forwards!
            }


            double current_heading = initial_heading;

            try {
                current_heading = normalizeDegrees(angles.heading);
            } catch (Exception e) { }

            double error = (current_heading - initial_heading);
            if (current_heading - initial_heading > 180)
                error = 360-error;

            error *= drive_straight_p_error_const;


            telemetry.addData("error", error);
            pl = motorPower - error;
            pr = motorPower + error;

            pl = scale(pl);
            pr = scale(pr);


            leftMotor.setPower(pl);
            rightMotor.setPower(pr);
            telemetry.update();

        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }


    void driveUntilTouch(double motorPower) {
        double initial_heading = 0;


        try {
            initial_heading = angles.heading;
        } catch(Exception e){}



        while (touch.getValue() == 0) {

            double current_heading = initial_heading;

            try {
                current_heading = normalizeDegrees(angles.heading);
            } catch (Exception e) {
            }

            double error = (current_heading - initial_heading);
            if (current_heading - initial_heading > 180)
                error = 360-error;


            telemetry.addData("error", error);

            error *= drive_straight_p_error_const;


            pl = motorPower - error;
            pr = motorPower + error;

            pl = scale(pl);
            pr = scale(pr);


            leftMotor.setPower(pl);
            rightMotor.setPower(pr);
            telemetry.update();

        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    double angle360(double angle) {
        return (angle > 0) ? angle : 360 + angle;
    }

    double scale(double d) {
        if (d > 1) return 1.0;
        if (d < -1) return -1.0;
        else return d;
    }

     double normalizeDegrees(double degrees)
    {
                while (degrees >= 180.0) degrees -= 360.0;
                while (degrees < -180.0) degrees += 360.0;
        return degrees;
    }
    double degreesFromRadians(double radians)
    {
        return radians * 180.0 / Math.PI;
    }

    void turnToHeading(double target_heading, double motorPower) {
        target_heading = angle360(target_heading);

        double current_heading = 0;

        try{
            current_heading = angle360(angles.heading);
        } catch(Exception e) {}


        double turnLeftDegrees = Math.abs(current_heading + 360 - target_heading);
        double turnRightDegrees = Math.abs(target_heading-current_heading);

        double degreeDiffThreshold = 5;


        double initDiff = Math.abs(current_heading - target_heading);


        if(turnLeftDegrees < turnRightDegrees) {
            pl = -motorPower;
            pr = motorPower;
        } else {
            pl = motorPower;
            pr = -motorPower;
        }


        long prevTime = System.nanoTime();


        do {
            telemetry.addData("time diff", (double) (System.nanoTime()-prevTime) / (double) 1000000000);
            prevTime = System.nanoTime();

            try {
                current_heading = angle360(angles.heading);
            } catch(Exception e) {};


            double scalingfactor = Math.abs(current_heading - target_heading)/initDiff;

            //            if(scalingfactor < .65)
            //                scalingfactor = .65;

            //            if(motorPower * scalingfactor < .3)
            //                scalingfactor = .3 / motorPower;

            //            pl *= scalingfactor;
            //            pr *= scalingfactor;

            leftMotor.setPower(pl);
            rightMotor.setPower(pr);


            telemetry.update();
        } while(Math.abs(current_heading - target_heading) > degreeDiffThreshold);

        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    void driveTicksStraight(double motorPower, int ticks) {

        int initialLeftPosition = leftMotor.getCurrentPosition();
        int initialRightPosition = rightMotor.getCurrentPosition();

        double initial_heading = 0;

        try {
            initial_heading = angles.heading;
        } catch(Exception e){}


        while (Math.abs(initialLeftPosition - leftMotor.getCurrentPosition()) < ticks
                && Math.abs(initialRightPosition - rightMotor.getCurrentPosition()) < ticks) {

            double current_heading = initial_heading;

            try {
                current_heading = normalizeDegrees(angles.heading);
            } catch (Exception e) {}


            double error = (current_heading - initial_heading);
            if (current_heading - initial_heading > 180)
                error = 360-error;

            error *= drive_straight_p_error_const;

            pl = motorPower - error;
            pr = motorPower + error;

            pl = scale(pl);
            pr = scale(pr);

            if(Math.abs(pl-pr) > 1) {
                pl = pr = motorPower;
            }


            leftMotor.setPower(pl);
            rightMotor.setPower(pr);
            telemetry.update();
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }