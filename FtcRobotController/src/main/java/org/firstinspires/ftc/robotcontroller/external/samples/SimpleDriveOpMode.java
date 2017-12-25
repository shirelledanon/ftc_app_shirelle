package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Shirelle on 12/13/2017.
 */

public class SimpleDriveOpMode extends OpMode  {

 DcMotor rightwheel ;
    DcMotor rightwheel1 ;
    DcMotor leftwheel1 ;
 DcMotor leftwheel;

     public void  init (){
         leftwheel = hardwareMap.dcMotor.get("leftwheelfront");
         leftwheel1 = hardwareMap.dcMotor.get("leftwheelback");
         rightwheel=hardwareMap.dcMotor.get("rightwheelfront");
         rightwheel1=hardwareMap.dcMotor.get("rightwheelback");

leftwheel.setDirection(DcMotor.Direction.REVERSE);
         leftwheel1.setDirection(DcMotor.Direction.REVERSE);
         rightwheel.setDirection(DcMotor.Direction.FORWARD);
         rightwheel1.setDirection(DcMotor.Direction.FORWARD);

     }


    public void loop() {

         double leftpower;
         double rightpower;
         double drive =gamepad1.right_stick_y;
         double turn= gamepad1.left_stick_x;

         if (drive!= 0) {
             leftpower= drive+turn ;
             leftwheel.setPower(leftpower);
             leftwheel1.setPower(leftpower);
             rightpower = drive -turn ;
             rightwheel.setPower(rightpower);
             rightwheel1.setPower(rightpower);


         }
         else  {
             leftpower = turn;
             rightpower= -turn;
             leftwheel.setPower(leftpower);
             leftwheel1.setPower(leftpower);
             rightwheel.setPower(rightpower);
             rightwheel1.setPower(rightpower);

        }











    }

    public void  stop () {


    }
}
