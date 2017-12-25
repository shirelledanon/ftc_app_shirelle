package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by talyo on 25/12/2017.
 */

@TeleOp(name = "TeleOp final")

public class TeleOpProg extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        RobotClass Robot = new RobotClass(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            Robot.Drive((gamepad1.left_stick_y - gamepad1.right_stick_x), (gamepad1.left_stick_y + gamepad1.right_stick_x));
            if (gamepad1.a)
                Robot.setSlidePower(1);
            if (gamepad1.b)
                Robot.setSlidePower(-1);
            if (gamepad1.a == false && gamepad1.b == false)
                Robot.setSlidePower(0);
        }
    }


}
