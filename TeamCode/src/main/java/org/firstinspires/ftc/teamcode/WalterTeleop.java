package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "Walter TeleOp", group = "Linear Opmode")
public class WalterTeleop extends LinearOpMode {

    private GustavoDriveTrain driveTrain;
    private static final GustavoDriveTrain.DirectionEnum direction = GustavoDriveTrain.DirectionEnum.SOUTH;

    @Override
    public void runOpMode() throws InterruptedException {
        this.driveTrain = new GustavoDriveTrain(this.hardwareMap, this.telemetry, direction);

        this.driveTrain.initialzeDriveMotors(hardwareMap);
        this.driveTrain.getHeading();
        this.driveTrain.reset_angle();

        waitForStart();


        while (opModeIsActive()) {

            double crabValue = 0.0;
            double moveValue = 0.0;
            double turnValue = 0.0;
            double maxPower;


            maxPower = 0.6;

            crabValue = gamepad1.left_stick_x;
            moveValue = gamepad1.left_stick_y;
            turnValue = gamepad1.right_stick_x;

            this.driveTrain.drive(crabValue, moveValue, turnValue, maxPower);
            idle();

//            crabValue = gamepad1.left_stick_x / 1.6;
//            moveValue = gamepad1.left_stick_y / 1.6;
//            turnValue = gamepad1.right_stick_x / 1.6;

//            if (gamepad1.right_stick_x > .4) {
//                turnValue = 0.5;
//            } else if (gamepad1.right_stick_x > -0.4) {
//                turnValue = -0.5;
//            } else if (gamepad1.left_stick_x > 0.4) {
//                crabValue = 0.5;
//            } else if (gamepad1.left_stick_x > -0.4) {
//                crabValue = -0.5;
//            } else if (gamepad1.left_stick_y > 0.4) {
//                moveValue = 0.5;
//            } else if (gamepad1.left_stick_y > -0.4) {
//                moveValue = -0.5;
//            }



//            this.driveTrain.drive(crabValue, moveValue, turnValue, maxPower);
//            idle();

            if(gamepad1.a) {
                this.driveTrain.reset_angle();
            }

            if(gamepad1.b) {
                this.driveTrain.stopNow();
            }




        }

    }
}
