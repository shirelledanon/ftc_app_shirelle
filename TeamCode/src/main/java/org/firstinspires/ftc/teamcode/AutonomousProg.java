package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by talyo on 19/12/2017.
 */
@Autonomous(name = "Autonomous final")
public class AutonomousProg extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {


        RobotClass Robot = new RobotClass(hardwareMap);

        waitForStart();
        runtime.reset();



        while (runtime.time() < 2.5) {
            double fix = Robot.PID(Robot.getGyroAngle(), 0, 0.013, 0.005, 0.005);
            Robot.Drive(-0.2 - fix, -0.2 + fix);
        }

        Robot.StopRobot(100);

        runtime.reset();
        while (runtime.time() < 1) {
            Robot.Drive(+0.3, -0.3);
        }
        Robot.StopRobot(100);

        runtime.reset();
        while (runtime.time() < 0.5) {
            double fix = Robot.PID(Robot.getGyroAngle(), 0, 0.013, 0.005, 0.005);
            Robot.Drive(0.2, 0.2 + fix);
        }

        Robot.StopRobot(100);

        Robot.setSlidePower(0.8);
        Thread.sleep(3000);


    }

}




