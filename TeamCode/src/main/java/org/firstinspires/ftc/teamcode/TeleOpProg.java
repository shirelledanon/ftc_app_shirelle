package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by talyo on 25/12/2017.
 */

@TeleOp(name = "TeleOp final")

public class TeleOpProg extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        RobotClass Robot = new RobotClass(hardwareMap);

        double X = 0.6;
        double powerRight;
        double powerLeft;
        waitForStart();
        while (opModeIsActive()) {

            powerRight = Range.clip(gamepad1.left_stick_y * 0.7 * X + gamepad1.right_stick_x * 0.4 * X + gamepad1.left_stick_x * 0.4 * X,
                    -0.8, 0.8);
            powerLeft =  Range.clip(gamepad1.left_stick_y * 0.7 * X - gamepad1.right_stick_x * 0.4 * X - gamepad1.left_stick_x * 0.4 * X,
                    -0.8, 0.8);
            Robot.Drive(powerRight, powerLeft);
            if (gamepad2.a)
                Robot.setSlidePower(-1);
            if (gamepad2.b)
                Robot.setSlidePower(1);
            if (gamepad2.a == false && gamepad2.b == false)
                Robot.setSlidePower(0);
            if (gamepad1.dpad_up)
                X = 0.6;
            if (gamepad1.dpad_down)
                X = 0.25;
            if(gamepad1.y)
                X = -X;
        }
    }


}
