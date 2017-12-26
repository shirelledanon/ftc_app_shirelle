package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by talyo on 25/12/2017.
 */
@Autonomous(name = "Try Drive")
public class DriveTry extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotClass Robot = new RobotClass(hardwareMap);
        waitForStart();


        Robot.DriveDistance(100, 0.4);
    }
}
