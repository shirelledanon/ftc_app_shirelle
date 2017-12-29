//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
//
///**
// * Created by Shirelle on 12/26/2017.
// */
//
//public class AutonimousRedTeam extends LinearOpMode {
//
//    ElapsedTime runtime = new ElapsedTime();
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        RelicRecoveryVuMark side = null;
//        runtime.startTime();
//        RobotClass Robot = new RobotClass(hardwareMap, telemetry, runtime);
//        waitForStart();
//        runtime.reset();
//
//        while (runtime.time() < 1.5) {
//            Robot.Drive(0, 0);
//            telemetry.addData("the visible vumark is", Robot.vuforia());
//            telemetry.update();
//            side = Robot.vuforia();
//        }
//        runtime.reset();
//        switch (side) {
//            case UNKNOWN:
//
//            case LEFT:{
//                runtime.reset();
//                Robot.DriveByCm(0.3, 40 );
//                Robot.Turn(0.3, -90);
//                Robot.DriveByCm(0.3, 70);
//                Robot.Turn(0.3, 90);
//                Robot.DriveByCm(0.3, 25);
//                runtime.reset();
//                while (runtime.time() < 1){
//                    Robot.setSlidePower(-1);
//
//                }
//              break;
//            }
//
//            case RIGHT:{
//                runtime.reset();
//                Robot.DriveByCm(0.3, 40 );
//                Robot.Turn(0.3, -90);
//                Robot.DriveByCm(0.3, 50);
//                Robot.Turn(0.3, 90);
//
//
//            }
//
//            case CENTER:{
//                Robot.DriveByCm(0.3, 40 );
//                Robot.Turn(0.3, -90);
//                Robot.DriveByCm(0.3, 30);
//
//
//
//            }
//
//
//        }
//
//
//
//    }
//    }
