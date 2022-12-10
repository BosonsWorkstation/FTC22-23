package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "The $uper cool t3l3Op", group = "Linear Opmode") //creates teleop
public class landdrivetrain extends LinearOpMode {

    protected DcMotor front_left_wheel = null;
    protected DcMotor front_right_wheel = null;
    protected DcMotor back_left_wheel = null;
    protected DcMotor back_right_wheel = null;


    @Override
    public void runOpMode() throws InterruptedException { //start of mycode

        front_left_wheel = hardwareMap.dcMotor.get("front_left_wheel");
        front_right_wheel = hardwareMap.dcMotor.get("front_right_wheel");
        back_left_wheel = hardwareMap.dcMotor.get("back_left_wheel");
        back_right_wheel = hardwareMap.dcMotor.get("back_right_wheel");
        int stop = 0;

        waitForStart();
        while (opModeIsActive()) { //what initiats the start program

            if(gamepad1.left_stick_x > -0.4) {  //lets the left stick turn the robot left when used
                front_left_wheel.setPower(-0.8);
                front_right_wheel.setPower(-0.8);
                back_left_wheel.setPower(-0.8);
                back_right_wheel.setPower(-0.8);
            }

            if(gamepad1.left_stick_x > 0.4) { //lets the left stick turn the robot right when used
                front_left_wheel.setPower(0.8);
                front_right_wheel.setPower(0.8);
                back_left_wheel.setPower(0.8);
                back_right_wheel.setPower(0.8);
            }

            //left stick above    ----------------------------------------------------------

            if (gamepad1.right_stick_x > -.4) { //lets the right stick move the robot left
                front_left_wheel.setPower(-.8);
                back_left_wheel.setPower(.8);
                front_right_wheel.setPower(-.8);
                back_right_wheel.setPower(.8);
            }
            else {
                front_left_wheel.setPower(stop);
                back_left_wheel.setPower(stop);
                front_right_wheel.setPower(stop);
                back_right_wheel.setPower(stop);
            }

            if (gamepad1.right_stick_x > .4) { //lets the right stick move the robot right
                front_left_wheel.setPower(.8);
                back_left_wheel.setPower(-.8);
                front_right_wheel.setPower(.8);
                back_right_wheel.setPower(-.8);
            }
            else {
                front_left_wheel.setPower(stop);
                back_left_wheel.setPower(stop);
                front_right_wheel.setPower(stop);
                back_right_wheel.setPower(stop);
            }

            if (gamepad1.right_stick_y > .4) { //lets the right stick move the robot forwards
                front_left_wheel.setPower(-.8);
                back_left_wheel.setPower(-.8);
                front_right_wheel.setPower(.8);
                back_right_wheel.setPower(.8);
            }
            else {
                front_left_wheel.setPower(stop);
                back_left_wheel.setPower(stop);
                front_right_wheel.setPower(stop);
                back_right_wheel.setPower(stop);
            }

            if (gamepad1.right_stick_y > -.4) { //lets the right stick move the robot backwards
                front_left_wheel.setPower(.8);
                back_left_wheel.setPower(.8);
                front_right_wheel.setPower(-.8);
                back_right_wheel.setPower(-.8);
            }
            else {
                front_left_wheel.setPower(stop);
                back_left_wheel.setPower(stop);
                front_right_wheel.setPower(stop);
                back_right_wheel.setPower(stop);
            }


            //right stick above ---------------------------------------------------------

                /*         if(gamepad1.right_stick_y > .4)
                            {
                             front_left_wheel.setPower(-.8);
                             back_left_wheel.setPower(-.8);
                                front_right_wheel.setPower(.8);
                             back_right_wheel.setPower(.8);
                                                 }                            */
        }





    }

}
