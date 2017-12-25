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



    }
}



