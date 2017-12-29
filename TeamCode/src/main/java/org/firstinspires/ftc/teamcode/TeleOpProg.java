package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by talyo on 25/12/2017.
 */

@TeleOp(name = "TeleOp final")

public class TeleOpProg extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime runtime = new ElapsedTime();
        RobotClass Robot = new RobotClass(hardwareMap,telemetry,runtime);

        double X = 0.55;
        double powerRight;
        double powerLeft;


        waitForStart();
        while (opModeIsActive()) {

            double glifpower = 0;
            powerRight = Range.clip((gamepad1.left_stick_y + gamepad1.right_stick_x) * X, -0.8, 0.8);
            powerLeft = Range.clip((gamepad1.left_stick_y - gamepad1.right_stick_x) * X, -0.8, 0.8);
            Robot.Drive(powerRight, powerLeft);
            if (gamepad2.left_trigger != 0) {
                glifpower = Range.clip(gamepad2.left_trigger, 0, -0.8);

            } else if (gamepad2.right_trigger != 0) {
                glifpower = Range.clip(gamepad2.right_trigger, 0, 0.8);
            }
            Robot.setSlidePower(glifpower);

            if (gamepad1.dpad_up)
                X = 0.55;
            if (gamepad1.dpad_down)
                X = 0.15;
            if (gamepad1.y)
                X = -X;
            if (gamepad2.a)
                break;






        }






        }
    }



