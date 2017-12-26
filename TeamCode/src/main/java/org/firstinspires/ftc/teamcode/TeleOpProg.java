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

        double X = 0.6;
        double powerRight;
        double powerLeft;

        waitForStart();
        while (opModeIsActive()) {

            powerRight = Range.clip((gamepad1.left_stick_y + gamepad1.right_stick_x) * X, -0.95, 0.95);
            powerLeft = Range.clip((gamepad1.left_stick_y - gamepad1.right_stick_x) * X, -0.95, 0.95);
            Robot.Drive(powerRight, powerLeft);
            double glifpower = 0;
            if (gamepad2.left_trigger != 0) {
                glifpower = Range.clip(gamepad2.left_trigger, 0, -0.8);

            } else if (gamepad2.right_trigger != 0) {
                glifpower = Range.clip(gamepad2.right_trigger, 0, 0.8);
            }
            Robot.setSlidePower(glifpower);

            double turn;




            if (gamepad1.dpad_up)
                X = 0.6;
            if (gamepad1.dpad_down)
                X = 0.25;
            if (gamepad1.y)
                X = -X;
        }
    }


}
