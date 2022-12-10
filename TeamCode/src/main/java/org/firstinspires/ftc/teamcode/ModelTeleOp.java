package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Model TeleOp", group = "Linear Opmode")
public class ModelTeleOp extends LinearOpMode {

    protected DcMotor front_left_wheel = null;
    protected DcMotor front_right_wheel = null;
    protected DcMotor back_left_wheel = null;
    protected DcMotor back_right_wheel = null;
    //protected Telemetry telemetry;


    @Override
    public void runOpMode() throws InterruptedException {

        front_left_wheel = hardwareMap.dcMotor.get("front_left_wheel");
        back_left_wheel = hardwareMap.dcMotor.get("back_left_wheel");
        front_right_wheel = hardwareMap.dcMotor.get("front_right_wheel");
        back_right_wheel = hardwareMap.dcMotor.get("back_right_wheel");

        int stop = 0;

        waitForStart();
        while (opModeIsActive()) {



            if (gamepad1.left_stick_x > .4) {
                front_left_wheel.setPower(-.8);
                back_left_wheel.setPower(-.8);
                front_right_wheel.setPower(-.8);
                back_right_wheel.setPower(-.8);
            } else {
                front_left_wheel.setPower(stop);
                back_left_wheel.setPower(stop);
                front_right_wheel.setPower(stop);
                back_right_wheel.setPower(stop);
            }
            if (gamepad1.left_stick_x > -.4) {
                front_left_wheel.setPower(.8);
                back_left_wheel.setPower(.8);
                front_right_wheel.setPower(.8);
                back_right_wheel.setPower(.8);
            } else {
                front_left_wheel.setPower(stop);
                back_left_wheel.setPower(stop);
                front_right_wheel.setPower(stop);
                back_right_wheel.setPower(stop);
            }
            if (gamepad1.right_stick_x < -.4) {
                front_left_wheel.setPower(-.8);
                back_left_wheel.setPower(.8);
                front_right_wheel.setPower(-.8);
                back_right_wheel.setPower(.8);
            } else {
                front_left_wheel.setPower(stop);
                back_left_wheel.setPower(stop);
                front_right_wheel.setPower(stop);
                back_right_wheel.setPower(stop);
            }
            if (gamepad1.right_stick_x > .4) {
                front_left_wheel.setPower(.8);
                back_left_wheel.setPower(-.8);
                front_right_wheel.setPower(.8);
                back_right_wheel.setPower(-.8);
            } else {
                front_left_wheel.setPower(stop);
                back_left_wheel.setPower(stop);
                front_right_wheel.setPower(stop);
                back_right_wheel.setPower(stop);
            }
            if (gamepad1.right_stick_y > .4) {
                front_left_wheel.setPower(-.8);
                back_left_wheel.setPower(-.8);
                front_right_wheel.setPower(.8);
                back_right_wheel.setPower(.8);
            } else {
                front_left_wheel.setPower(stop);
                back_left_wheel.setPower(stop);
                front_right_wheel.setPower(stop);
                back_right_wheel.setPower(stop);
            }
            if (gamepad1.right_stick_y < -.4) {
                front_left_wheel.setPower(.8);
                back_left_wheel.setPower(.8);
                front_right_wheel.setPower(-.8);
                back_right_wheel.setPower(-.8);
            } else {
                front_left_wheel.setPower(stop);
                back_left_wheel.setPower(stop);
                front_right_wheel.setPower(stop);
                back_right_wheel.setPower(stop);
            }

//            telemetry.addData("front_left_wheel", front_left_wheel.getPower());
//            telemetry.addData("back_left_wheel", back_left_wheel.getPower());
//            telemetry.addData("front_right_wheel", front_right_wheel.getPower());
//            telemetry.addData("back_right_wheel", back_right_wheel.getPower());
//            telemetry.update();



        }
    }
}
