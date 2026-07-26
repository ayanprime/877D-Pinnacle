#include "main.h"

void setFlipper() {

    flipper.setVelocity(60, pct);
    flipper.spinToPosition(105, degrees);

}

void flipFlipper() {

    flipper.setVelocity(15, pct);
    flipper.spinToPosition(-70, degrees);

}

void AyanDrive() {

    lfm.spin(forward, ControllerDriver.Axis3.position() + ControllerDriver.Axis1.position(), pct);
    lmm.spin(forward, ControllerDriver.Axis3.position() + ControllerDriver.Axis1.position(), pct);
    lbm.spin(forward, ControllerDriver.Axis3.position() + ControllerDriver.Axis1.position(), pct);

    rfm.spin(forward, ControllerDriver.Axis3.position() - ControllerDriver.Axis1.position(), pct);
    rmm.spin(forward, ControllerDriver.Axis3.position() - ControllerDriver.Axis1.position(), pct);
    rbm.spin(forward, ControllerDriver.Axis3.position() - ControllerDriver.Axis1.position(), pct);

    ControllerDriver.ButtonY.pressed(flipFlipper);
    ControllerDriver.ButtonX.pressed(setFlipper);

    if (ControllerDriver.ButtonL1.pressing()) {

        intake.spin(forward, 100, pct);

    } else if (ControllerDriver.ButtonL2.pressing()) {

        intake.spin(reverse, 100, pct);

    } else {

        intake.stop();

    }

}