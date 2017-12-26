package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by talyo on 19/12/2017.
 */
@Autonomous(name = "Autonomous final")
public class AutonomousProg extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {

        RelicRecoveryVuMark side = null;
        runtime.startTime();
        RobotClass Robot = new RobotClass(hardwareMap, telemetry, runtime);
        waitForStart();
        runtime.reset();

        while (runtime.time() < 1.5) {
            Robot.Drive(0, 0);
            telemetry.addData("the visible vumark is", Robot.vuforia());
            telemetry.update();
            side = Robot.vuforia();
        }
        runtime.reset();
        switch (side) {
            case UNKNOWN:

            case LEFT:
                runtime.reset();
                Robot.DriveByTime(0.3, 0.3, 1000);
            case RIGHT:
                runtime.reset();
                Robot.DriveByTime(0.3, 0.3, 500);
            case CENTER:
                runtime.reset();
                while (runtime.time() < 0.75)
                    Robot.Drive(0.3, 0.3);


        }


        Robot.StopRobot(100);
        Robot.DriveByTime(0.3, -0.3, 400);
        Robot.StopRobot(100);
        Robot.DriveByTime(0.3, 0.3, 400);
        Robot.StopRobot(100);
        Robot.setSlidePower(0.8);
    }
}



