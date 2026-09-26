#include "main.h"

void setFlipper() {

    flipper.setVelocity(60, pct);
    flipper.spinToPosition(105, degrees);

}

void flipFlipper() {

    flipper.setVelocity(15, pct);
    flipper.spinToPosition(-70, degrees);

}

void levelUp() {

    if (Level < 8) {

        Level += 1;

    }

}

void levelDown() {

    if (Level > 0) {

        Level -= 1;

    }

}

void resetLevel() {

    Level = 0;

}

void AyanDrive() {

    antiTip();
    
    lfm.spin(forward, ControllerDriver.Axis3.position() + ControllerDriver.Axis1.position(), pct);
    lmm.spin(forward, ControllerDriver.Axis3.position() + ControllerDriver.Axis1.position(), pct);
    lbm.spin(forward, ControllerDriver.Axis3.position() + ControllerDriver.Axis1.position(), pct);

    rfm.spin(forward, ControllerDriver.Axis3.position() - ControllerDriver.Axis1.position(), pct);
    rmm.spin(forward, ControllerDriver.Axis3.position() - ControllerDriver.Axis1.position(), pct);
    rbm.spin(forward, ControllerDriver.Axis3.position() - ControllerDriver.Axis1.position(), pct);

    ControllerDriver.ButtonY.pressed(flipFlipper);
    ControllerDriver.ButtonX.pressed(setFlipper);

    if (ControllerDriver.ButtonR1.pressing()) {

        intake.spin(forward, 100, pct);

    } else if (ControllerDriver.ButtonR2.pressing()) {

        intake.spin(reverse, 100, pct);

    } else {

        intake.stop();

    }

    ControllerDriver.ButtonUp.pressed(levelUp);
    ControllerDriver.ButtonDown.pressed(levelDown);
    ControllerDriver.ButtonB.pressed(resetLevel);

}

void KimmyDrive() {

    lfm.spin(forward, ControllerDriver.Axis3.position() + ControllerDriver.Axis4.position() + ControllerDriver.Axis1.position(), pct);
    lmm.spin(forward, ControllerDriver.Axis3.position() + ControllerDriver.Axis4.position(), pct);
    lbm.spin(forward, ControllerDriver.Axis3.position() + ControllerDriver.Axis4.position() - ControllerDriver.Axis1.position(), pct);

    rfm.spin(forward, ControllerDriver.Axis3.position() - ControllerDriver.Axis4.position() - ControllerDriver.Axis1.position(), pct);
    rmm.spin(forward, ControllerDriver.Axis3.position() - ControllerDriver.Axis4.position(), pct);
    rbm.spin(forward, ControllerDriver.Axis3.position() - ControllerDriver.Axis4.position() + ControllerDriver.Axis1.position(), pct);

}
