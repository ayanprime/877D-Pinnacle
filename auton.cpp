#include "main.h"

// Robot Position

// Default testing position

double robotX = 0;
double robotY = 0;

// Angle Functions

double wrapAngle(double angle) {
    
    while(angle > 180) {
        
        angle -= 360;

    }

    while(angle < -180) {

        angle += 360;

    }

    return angle;

}

// GPS Coordinate Reading
// Call only when GPS is available

void GetCoordinate() {

    robotX = GPS.xPosition(inches);
    robotY = GPS.yPosition(inches);

}


// Asterisk Drive Turning

void setTurn(double power) {
    
    // Left motors
    lfm.spin(forward, power, percent);
    lmm.spin(forward, power, percent);
    lbm.spin(forward, power, percent);


    // Right motors
    rfm.spin(reverse, power, percent);
    rmm.spin(reverse, power, percent);
    rbm.spin(reverse, power, percent);

}



void stopDrive() {

    lfm.stop(brake);
    lmm.stop(brake);
    lbm.stop(brake);

    rfm.stop(brake);
    rmm.stop(brake);
    rbm.stop(brake);

}

// PID Turn Controller

void turnToHeading(double target, double kP, double kI, double kD) {
    
    double error = 0;
    double previousError = 0;

    double integral = 0;
    double derivative = 0;


    int settled = 0;



    while(settled < 15) {

        // Sensor value

        double current = inrtl.rotation(degrees);

        // Error = target - current

        error = wrapAngle(target - current);

        // Integral

        integral += error;

        // Reset integral if we cross target

        if((error > 0 && previousError < 0) || (error < 0 && previousError > 0)) {
            
            integral = 0;

        }

        // Integral windup protection

        if(integral > 1000) {
            
            integral = 1000;

        }

        if(integral < -1000) {
            
            integral = -1000;

        }

        // Derivative

        derivative = error - previousError;

        // PID Calculation

        double power = (error * kP) + (integral * kI) + (derivative * kD);

        // Limit output

        if(power > 100) {

            power = 100;

        }
        
        if(power < -100) {

            power = -100;

        }

        // Minimum torque
        // only when far from target

        if(fabs(error) > 3) {
            
            if(power > 0 && power < 8) {
                
                power = 8;

            }

            if(power < 0 && power > -8) {

                power = -8;

            }

        }

        setTurn(power);
        
        // Settling Check

        if(fabs(error) < 0.3) {
            
            settled++;

        } else {
            
            settled = 0;

        }



        previousError = error;



        wait(15, msec);

    }

    stopDrive();

}

// Turn To Coordinate

void turnToPoint(double targetX, double targetY, double kP, double kI, double kD) {

    // Calculate angle from robot
    // position to target

    double targetAngle = atan2(targetY - robotY, targetX - robotX) * 180 / 3.14159265359;


    if(targetAngle < 0) {
    
        targetAngle += 360;

    }

    turnToHeading(targetAngle, kP, kI, kD);

}

// Example Auto

void LRQ() {

    // Testing without GPS
    // Robot starts at 0,0

    robotX = 0;
    robotY = 0;



    // Example:
    // Face coordinate (30,60)

    turnToPoint(30, 60, 0.8, 0, 0.5);

    // Competition example:
    //
    // GetCoordinate();
    //
    // turnToPoint(
    //     30,
    //     60,
    //     0.5,
    //     0,
    //     2.5
    // );

}
