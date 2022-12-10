package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

//import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "SaulOvers", group = "Linear Opmode")
public class SaulOver extends LinearOpMode {

    protected DcMotor front_left_wheel = null;
    protected DcMotor front_right_wheel = null;
    protected DcMotor back_left_wheel = null;
    protected DcMotor back_right_wheel = null;
    protected DcMotor arm = null;

    protected Servo armServo = null;
    //protected Telemetry telemetry;


    @Override
    public void runOpMode() throws InterruptedException {

        front_left_wheel = hardwareMap.dcMotor.get("front_left_wheel");
        back_left_wheel = hardwareMap.dcMotor.get("back_left_wheel");
        front_right_wheel = hardwareMap.dcMotor.get("front_right_wheel");
        back_right_wheel = hardwareMap.dcMotor.get("back_right_wheel");
        arm = hardwareMap.dcMotor.get("arm");

        armServo = hardwareMap.servo.get("armServo");



        int stop = 0;
        double power = 0.4;
        double armSpeed = 0.3;

        boolean armServoClosed;

        waitForStart();
        while (opModeIsActive()) {


            if (gamepad1.left_stick_x < -.4) {
                front_left_wheel.setPower(power);
                back_left_wheel.setPower(power);
                front_right_wheel.setPower(power);
                back_right_wheel.setPower(power);
            } else {
                front_left_wheel.setPower(stop);
                back_left_wheel.setPower(stop);
                front_right_wheel.setPower(stop);
                back_right_wheel.setPower(stop);
            }
            if (gamepad1.left_stick_x > .4) {
                front_left_wheel.setPower(-power);
                back_left_wheel.setPower(-power);
                front_right_wheel.setPower(-power);
                back_right_wheel.setPower(-power);
            } else {
                front_left_wheel.setPower(stop);
                back_left_wheel.setPower(stop);
                front_right_wheel.setPower(stop);
                back_right_wheel.setPower(stop);
            }
            if (gamepad1.right_stick_x < -.4) {
                front_left_wheel.setPower(-power);
                back_left_wheel.setPower(power);
                front_right_wheel.setPower(power);
                back_right_wheel.setPower(-power);
            } else {
                front_left_wheel.setPower(stop);
                back_left_wheel.setPower(stop);
                front_right_wheel.setPower(stop);
                back_right_wheel.setPower(stop);
            }
            if (gamepad1.right_stick_x > .4) {
                front_left_wheel.setPower(power);
                back_left_wheel.setPower(-power);
                front_right_wheel.setPower(-power);
                back_right_wheel.setPower(power);
            } else {
                front_left_wheel.setPower(stop);
                back_left_wheel.setPower(stop);
                front_right_wheel.setPower(stop);
                back_right_wheel.setPower(stop);
            }
            if (gamepad1.right_stick_y > .4) {
                front_left_wheel.setPower(-power);
                back_left_wheel.setPower(-power);
                front_right_wheel.setPower(power);
                back_right_wheel.setPower(power);
            } else {
                front_left_wheel.setPower(stop);
                back_left_wheel.setPower(stop);
                front_right_wheel.setPower(stop);
                back_right_wheel.setPower(stop);
            }
            if (gamepad1.right_stick_y < -.4) {
                front_left_wheel.setPower(power);
                back_left_wheel.setPower(power);
                front_right_wheel.setPower(-power);
                back_right_wheel.setPower(-power);
            } else {
                front_left_wheel.setPower(stop);
                back_left_wheel.setPower(stop);
                front_right_wheel.setPower(stop);
                back_right_wheel.setPower(stop);
            }



            if(gamepad1.dpad_up) {
                arm.setPower(armSpeed);
            } else if(gamepad1.dpad_down) {
                arm.setPower(-armSpeed);
            } else {
                arm.setPower(0);
            }

            if (gamepad1.right_bumper) {

                armServo.setPosition(0.5);
            }
            if (gamepad1.left_bumper) {
                armServo.setPosition(0.2);
            }
        }
    }
}

//            telemetry.addData("front_left_wheel", front_left_wheel.getPower());
//            telemetry.addData("back_left_wheel", back_left_wheel.getPower());
//            telemetry.addData("front_right_wheel", front_right_wheel.getPower());
//            telemetry.addData("back_right_wheel", back_right_wheel.getPower());
//            telemetry.update();


