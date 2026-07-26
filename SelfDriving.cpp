#include "main.h"

void antiTip() {

    if (inrtl.pitch() >= 38) {

        lfm.spin(forward, 100, pct);
        lmm.spin(forward, 100, pct);
        lbm.spin(forward, 100, pct);

        rfm.spin(forward, 100, pct);
        rmm.spin(forward, 100, pct);
        rbm.spin(forward, 100, pct);

    } else if (inrtl.pitch() <= -32) {

        lfm.spin(reverse, 100, pct);
        lmm.spin(reverse, 100, pct);
        lbm.spin(reverse, 100, pct);

        rfm.spin(reverse, 100, pct);
        rmm.spin(reverse, 100, pct);
        rbm.spin(reverse, 100, pct);

    }

}