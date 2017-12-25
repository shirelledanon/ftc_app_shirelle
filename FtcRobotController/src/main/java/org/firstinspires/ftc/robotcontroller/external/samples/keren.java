package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Shirelle on 12/21/2017.
 */

public class keren extends OpMode {

    DcMotor rightwheel ;
    DcMotor rightwheel1 ;
    DcMotor leftwheel1 ;
    DcMotor leftwheel;
    DcMotor leftrow;
    DcMotor rightrow ;


    public void  init (){
        leftwheel = hardwareMap.dcMotor.get("leftwheelfront");
        leftwheel1 = hardwareMap.dcMotor.get("leftwheelback");
        rightwheel=hardwareMap.dcMotor.get("rightwheelfront");
        rightwheel1=hardwareMap.dcMotor.get("rightwheelback");
        leftrow = hardwareMap.dcMotor.get("left_row");
        rightrow = hardwareMap.dcMotor.get("right_row");

        leftwheel.setDirection(DcMotor.Direction.REVERSE);
        leftwheel1.setDirection(DcMotor.Direction.REVERSE);
        rightwheel.setDirection(DcMotor.Direction.FORWARD);
        rightwheel1.setDirection(DcMotor.Direction.FORWARD);

    }


    public void loop() {

        double leftpower=0;
        double rightpower=0;
        double drive =gamepad1.right_stick_y;
        double turn= gamepad1.left_stick_x ;
        double rowpower = 0;

        if (gamepad2.a) {
            rowpower = 1;
            leftrow.setPower(rowpower);
            rightrow.setPower(rowpower);
        }

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






}
