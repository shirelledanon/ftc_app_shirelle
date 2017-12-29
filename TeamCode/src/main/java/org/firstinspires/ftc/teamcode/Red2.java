package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by Shirelle on 12/27/2017.
 */
@Autonomous(name = "Red 2")
public class Red2 extends LinearOpMode {
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
        Robot.DriveByTime(0.2, 0.2, 1300);
        Robot.DriveByTime(-0.2, -0.2, 1600);
        Robot.StopRobot(100);
        runtime.reset();
        switch (side) {

            case UNKNOWN:
                runtime.reset();
                Robot.DriveByCm(-0.3,-14);

            case LEFT:
                runtime.reset();
                Robot.DriveByCm(-0.3,-25);
                Robot.Turn(-0.3,25,1.45);

                break;
            case RIGHT:
                runtime.reset();
                Robot.Turn(-0.3,25,-1.45);

                break;
            case CENTER:
                runtime.reset();
                Robot.DriveByCm(-0.3,-16);
                Robot.Turn(-0.3,25,-1.45);


                break;
        }
        Robot.DriveByTime(+0.3,+0.3,2000);
        Robot.DriveByCm(0.2,9);
        runtime.reset();
        while (runtime.time() < 4)
            Robot.setSlidePower(1);





    }
}

