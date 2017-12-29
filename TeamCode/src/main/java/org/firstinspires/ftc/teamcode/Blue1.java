package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by Shirelle on 12/27/2017.
 */
@Autonomous(name = "Blue 1")
public class Blue1 extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {

        RelicRecoveryVuMark side = null;
        runtime.startTime();
        RobotClass Robot = new RobotClass(hardwareMap, telemetry, runtime);
        waitForStart();
        runtime.reset();

        while (runtime.time() < 3) {
            Robot.Drive(0, 0);
            telemetry.addData("the visible vumark is", Robot.vuforia());
            telemetry.update();
            side = Robot.vuforia();
        }
        Robot.DriveByTime(-0.15, -0.15, 1300);
        Robot.DriveByTime(0.2, 0.2, 1600);
        Robot.StopRobot(100);
        runtime.reset();
        switch (side) {

            case UNKNOWN:

                Robot.DriveByCm(0.3, 37);
                break;
            case LEFT:

                Robot.DriveByCm(0.3, 55);
                break;
            case RIGHT:

                Robot.DriveByCm(0.3, 17);
                break;
            case CENTER:

                Robot.DriveByCm(0.3, 37);
                break;
        }


        Robot.Turn(-0.3, 86, 1.27);
        Robot.StopRobot(100);
        Robot.DriveByCm(0.3, 25);
        runtime.reset();
        while (runtime.time() < 4) {
            Robot.setSlidePower(-1);
        }


    }
}
