#include "main.h"

double selfDrivingTargetX = 0;
double selfDrivingTargetY = 0;
double selfDrivingTargetHeading = 0;

/*
fwdTrack  must count UP when the robot drives forward
sideTrack must count UP when the robot moves right 
*/

char selfDrivingDecision[48] = "Idle";

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

    curX = GPS.xPosition(inches);
    curY = GPS.yPosition(inches);

}

void SetHome(){

    GetCoordinate();
    if (curX <= -23.6 && curY >= 0) {

        homeX = 59.05;
        homeY = 59.05;

    } else if(curX <= -23.6 && curY <= 0) {

        homeX = 59.05;
        homeY = -59.05;

    } else if(curX >= 23.6 && curY >= 0) {

        homeX = -59.05;
        homeY = 59.05;

    } else if(curX >= 23.6 && curY <= 0) {

        homeX = -59.05;
        homeY = -59.05;

    }

}


// Odometry (odom pods)

const double TRACK_DIAM   = 1.75;                // inches (calibrate with the 48 inch push test)
const double TRACK_CIRC   = TRACK_DIAM * M_PI;   // inches per pod revolution
const double FWD_WHEEL_X  = 0.0;   // forward pod: inches to the RIGHT of robot center (negative = left). Set from calibrateOffsets()
const double SIDE_WHEEL_Y = 0.0;   // side pod: inches FORWARD of robot center (negative = behind). Set from calibrateOffsets()

double headingRad() {

    return inrtl.rotation(degrees) * M_PI / 180.0;

}

int odomLoop() {

    double lastF  = fwdTrack.position(degrees)  / 360.0 * TRACK_CIRC;
    double lastS  = sideTrack.position(degrees) / 360.0 * TRACK_CIRC;
    double lastTh = headingRad();

    while(true) {

        double F  = fwdTrack.position(degrees)  / 360.0 * TRACK_CIRC;
        double S  = sideTrack.position(degrees) / 360.0 * TRACK_CIRC;
        double th = headingRad();

        double dF  = F - lastF;
        double dS  = S - lastS;
        double dTh = th - lastTh;

        lastF = F;
        lastS = S;
        lastTh = th;

        // remove the part of each reading caused by the robot turning
        dF += dTh * FWD_WHEEL_X;
        dS -= dTh * SIDE_WHEEL_Y;

        double avg = th - dTh / 2.0;   // average heading over this step

        // heading: 0 = +Y, clockwise positive
        curX += dS * cos(avg) + dF * sin(avg);
        curY += -dS * sin(avg) + dF * cos(avg);

        wait(10, msec);

    }

    return 0;

}

task* odomTask = NULL;

// Call once after the inertial sensor finishes calibrating and
// after curX / curY have their starting values.
void startOdometry() {

    fwdTrack.resetPosition();
    sideTrack.resetPosition();

    if(odomTask == NULL) odomTask = new task(odomLoop);

}

// Asterisk Drive Kinematics

// Drivetrain geometry:
const double D_MID    = 3.25;          // middle wheel diameter, inches
const double D_CORNER = 3.25;          // corner wheel diameter, inches
const double G_MID    = 60.0 / 36.0;   // wheel rpm per motor rpm (60 driving : 36 driven)
const double G_CORNER = 36.0 / 48.0;   // wheel rpm per motor rpm (36 driving : 48 driven)
const double R_MID    = 7.5892935;     // inches, robot center -> middle wheel contact
const double R_CORNER = 9.314995;      // inches, robot center -> corner wheel contact
const double S45      = 0.7071;

// vx = right, vy = forward, w = clockwise.
// All in percent of the max the drivetrain can do.

void driveVector(double vx, double vy, double w) {

    //                            lfm       lmm      lbm       rfm        rmm       rbm
    const double tx[6]     = {    S45,       0,     -S45,     -S45,        0,       S45 };
    const double ty[6]     = {    S45,       1,      S45,      S45,        1,       S45 };
    const double rr[6]     = { R_CORNER,  R_MID, R_CORNER, -R_CORNER, -R_MID, -R_CORNER };
    const double dia[6]    = { D_CORNER, D_MID,  D_CORNER,  D_CORNER,  D_MID,  D_CORNER };
    const double gear[6]   = { G_CORNER, G_MID,  G_CORNER,  G_CORNER,  G_MID,  G_CORNER };
    const double maxRpm[6] = {      600,   200,       600,       600,    200,       600 };

    double K[6];   // motor rpm per inch/sec of wheel surface speed
    double vMax = 1e9;
    double wMax = 1e9;

    for(int i = 0; i < 6; i++) {

        K[i] = 60.0 / (M_PI * dia[i] * gear[i]);

        double smax = maxRpm[i] / K[i];   // fastest this wheel's surface can go

        if(fabs(ty[i]) > 1e-9 && smax / fabs(ty[i]) < vMax) vMax = smax / fabs(ty[i]);
        if(fabs(rr[i]) > 1e-9 && smax / fabs(rr[i]) < wMax) wMax = smax / fabs(rr[i]);

    }

    double vxs = vx / 100.0 * vMax;   // inches/sec
    double vys = vy / 100.0 * vMax;
    double ws  = w  / 100.0 * wMax;   // radians/sec

    double T[6];
    double R[6];
    double rs = 1.0;

    for(int i = 0; i < 6; i++) {

        T[i] = K[i] * (tx[i] * vxs + ty[i] * vys);   // motor rpm for moving
        R[i] = K[i] * rr[i] * ws;                    // motor rpm for turning

        if(fabs(R[i]) / maxRpm[i] > rs) rs = fabs(R[i]) / maxRpm[i];

    }

    for(int i = 0; i < 6; i++) R[i] /= rs;

    // Turning keeps priority: if a motor runs out of speed,
    // translation shrinks first so heading never gets squashed.
    double s = 1.0;

    for(int i = 0; i < 6; i++) {

        if(fabs(T[i]) > 1e-9) {

            double lim = (maxRpm[i] - (T[i] > 0 ? R[i] : -R[i])) / fabs(T[i]);

            if(lim < s) s = lim;

        }

    }

    if(s < 0) s = 0;

    lfm.spin(forward, T[0] * s + R[0], rpm);
    lmm.spin(forward, T[1] * s + R[1], rpm);
    lbm.spin(forward, T[2] * s + R[2], rpm);
    rfm.spin(forward, T[3] * s + R[3], rpm);
    rmm.spin(forward, T[4] * s + R[4], rpm);
    rbm.spin(forward, T[5] * s + R[5], rpm);

}

// Asterisk Drive Turning

// power = percent of the fastest spin the drivetrain can do.
// Positive = clockwise.

void setTurn(double power) {

    driveVector(0, 0, power);

}



void stopDrive() {

    lfm.stop(brake);
    lmm.stop(brake);
    lbm.stop(brake);

    rfm.stop(brake);
    rmm.stop(brake);
    rbm.stop(brake);

}



// Run once with the robot on blocks to check directions
void testDirections() {

    driveVector(0, 40, 0);  wait(1000, msec);   // should drive forward
    driveVector(40, 0, 0);  wait(1000, msec);   // should strafe right
    driveVector(0, 0, 40);  wait(1000, msec);   // should spin clockwise
    stopDrive();

}

// Temporary helper: spins the robot 10 times and prints the two pod offsets.
// Run it on its own, without startOdometry(), because it resets the pods.
void calibrateOffsets() {

    fwdTrack.resetPosition();
    sideTrack.resetPosition();

    double start = inrtl.rotation(degrees);
    while(inrtl.rotation(degrees) - start < 3600) { setTurn(20); wait(10, msec); }
    stopDrive();
    wait(500, msec);

    double F = fwdTrack.position(degrees)  / 360.0 * TRACK_CIRC;
    double S = sideTrack.position(degrees) / 360.0 * TRACK_CIRC;
    double angle = (inrtl.rotation(degrees) - start) * M_PI / 180.0;   // radians turned

    Brain.Screen.clearScreen();
    Brain.Screen.printAt(10, 40, "FWD_WHEEL_X  = %.2f", -F / angle);
    Brain.Screen.printAt(10, 70, "SIDE_WHEEL_Y = %.2f",  S / angle);

}

// PID Turn Controller

void turnToHeading(double target, double kP, double kI, double kD) {

    selfDrivingTargetHeading = target;
    snprintf(selfDrivingDecision, sizeof(selfDrivingDecision), "Turning to %.0f deg", target);

    double startX = curX;   // a turn in place does not change position
    double startY = curY;

    double error = 0;
    double previousError = wrapAngle(target - inrtl.rotation(degrees)); // no derivative kick
    double integral = 0;
    double derivative = 0;
    int settled = 0;
    int loops = 0;

    while(settled < 15 && loops < 200) {   // 200 loops x 15 ms = 3 s timeout

        double current = inrtl.rotation(degrees);
        error = wrapAngle(target - current);

        integral += error;
        if((error > 0 && previousError < 0) || (error < 0 && previousError > 0)) integral = 0;
        if(integral > 1000) integral = 1000;
        if(integral < -1000) integral = -1000;

        derivative = error - previousError;

        double power = (error * kP) + (integral * kI) + (derivative * kD);
        if(power > 100) power = 100;
        if(power < -100) power = -100;

        if(fabs(error) > 3) {
            if(power > 0 && power < 8) power = 8;
            if(power < 0 && power > -8) power = -8;
        }

        setTurn(power);

        if(fabs(error) < 0.3) settled++; else settled = 0;

        previousError = error;
        loops++;
        wait(15, msec);

    }

    snprintf(selfDrivingDecision, sizeof(selfDrivingDecision), "%s", settled >= 15 ? "Turn done" : "Turn timed out");

    stopDrive();

    wait(150, msec);   // let the pods stop rolling
    curX = startX;     // throw away the fake distance the pods read during the turn
    curY = startY;

}

// Turn To Coordinate

void turnToPoint(double targetX, double targetY, double kP, double kI, double kD) {

    selfDrivingTargetX = targetX;
    selfDrivingTargetY = targetY;

    double dx = targetX - curX;
    double dy = targetY - curY;
    double targetHeading = atan2(dx, dy) * 180.0 / M_PI;  // 0° = +Y, clockwise-positive

    turnToHeading(targetHeading, kP, kI, kD);

}

// Move To Coordinate (drives and turns at the same time)

const double HEAD_KP = 1.2;         // heading hold while moving
const double HEAD_KD = 0.3;
const int MOVE_TIMEOUT_MS = 6000;

void moveToPoint(double tx, double ty, double kP, double kI, double kD, double finalAngle) {

    selfDrivingTargetX = tx;
    selfDrivingTargetY = ty;
    selfDrivingTargetHeading = finalAngle;

    double prevDist = hypot(tx - curX, ty - curY);
    double prevH = wrapAngle(finalAngle - inrtl.rotation(degrees));
    double integral = 0;
    double lastSpeed = 0;
    int settled = 0;
    int t = 0;

    double closestDist = prevDist;
    int movingAwayLoops = 0;
    bool lost = false;  

    while(settled < 10 && t < MOVE_TIMEOUT_MS) {

        double dx = tx - curX;
        double dy = ty - curY;
        double dist = hypot(dx, dy);

        if(dist < closestDist) { closestDist = dist; movingAwayLoops = 0; }
        else if(dist > closestDist + 4.0) movingAwayLoops++;
        if(movingAwayLoops > 10) { lost = true; break; }

        const char* decision = "Driving to target";
        if(dist < 3.0) decision = "Slowing down";
        if(dist < 0.7) decision = "Aligning heading";
        snprintf(selfDrivingDecision, sizeof(selfDrivingDecision), "%s", decision);

        double th = headingRad();

        // field direction -> robot frame
        double lx = dx * cos(th) - dy * sin(th);   // right
        double ly = dx * sin(th) + dy * cos(th);   // forward

        // PID on distance
        if(dist < 12) integral += dist; else integral = 0;
        if(integral > 300) integral = 300;

        double speed = dist * kP + integral * kI + (dist - prevDist) * kD;
        if(speed < 0) speed = 0;
        if(speed > lastSpeed + 6) speed = lastSpeed + 6;   // ramps up so wheels don't slip
        if(dist > 0.5 && speed < 8) speed = 8;             // enough power to beat friction
        lastSpeed = speed;

        double ux = 0;
        double uy = 0;
        if(dist > 1e-6) { ux = lx / dist; uy = ly / dist; }

        // turn toward the final angle while driving
        double hErr = wrapAngle(finalAngle - inrtl.rotation(degrees));
        double w = hErr * HEAD_KP + (hErr - prevH) * HEAD_KD;
        if(w > 40) w = 40;
        if(w < -40) w = -40;

        driveVector(ux * speed, uy * speed, w);

        settled = (dist < 0.5 && fabs(hErr) < 1.5) ? settled + 1 : 0;

        prevDist = dist;
        prevH = hErr;
        wait(15, msec);
        t += 15;

    }

    snprintf(selfDrivingDecision, sizeof(selfDrivingDecision), "%s", lost ? "Lost position" : (settled >= 10 ? "Arrived" : "Timed out"));

    stopDrive();

}

// Example Autonomous

void Auto() {
    
    curX =-59.25;
    curY = 58.0;
    inrtl.setHeading(180, degrees);

    if (curX <= -1 && curY >= 0) {

        moveToPoint(-48, 48, 3, 0, 0, 180);
        blueCont.spinToPosition(5, turns, false);
        pinkCont.spinToPosition(5, turns, true);
        wait(0.5, sec);
        moveToPoint(-48, 30, 3, 0, 0, 180);
        blueCont.spinToPosition(0, turns, false);
        pinkCont.spinToPosition(0, turns, true);
        claw.set(!(claw.value()));

    } else if(curX <= -1 && curY <= 0) {

    

    } else if(curX >= 1 && curY >= 0) {

       

    } else if(curX >= 1 && curY <= 0) {

        

    }
    
}
