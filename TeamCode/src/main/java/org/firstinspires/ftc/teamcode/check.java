package org.firstinspires.ftc.teamcode;

import android.hardware.SensorManager;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Shirelle on 12/26/2017.
 */
@Autonomous(name = "check")
public class check extends LinearOpMode {
ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        RobotClass Robot = new RobotClass(hardwareMap,telemetry,runtime);
        waitForStart();

        Robot.Turn(0.3,90);
    }
}

