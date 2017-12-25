package org.firstinspires.ftc.robotcontroller.external.samples.SIMPLEDRIVEOPMODE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Shirelle on 12/21/2017.
 */

public class Relic extends OpMode {

    Servo Relic;
    double position;
    @Override
    public void init() {

      Relic = hardwareMap.servo.get("Relic");
    }

    @Override
    public  void  loop () {

        position = 0.1 *gamepad2.left_stick_y;
        Relic.setPosition(position);
    }
}