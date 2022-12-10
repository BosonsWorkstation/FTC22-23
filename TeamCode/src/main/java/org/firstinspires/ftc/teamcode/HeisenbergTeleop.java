package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "Heisenberg TeleOp", group = "Linear Opmode")
public class HeisenbergTeleop extends LinearOpMode {

    private SalamancaDriveTrain driveTrain;
    private static final GustavoDriveTrain.DirectionEnum direction = GustavoDriveTrain.DirectionEnum.SOUTH;

    @Override
    public void runOpMode() throws InterruptedException {
        this.driveTrain = new SalamancaDriveTrain(this.hardwareMap, this.telemetry, direction);

        this.driveTrain.initialzeDriveMotors(hardwareMap);
        //this.driveTrain.getHeading();
        //this.driveTrain.reset_angle();

        waitForStart();


            while (opModeIsActive()) {

                double crabValue = 0.0;
                double moveValue = 0.0;
                double turnValue = 0.0;
                double maxPower;


                maxPower = 0.6;

                double y = -gamepad1.left_stick_y; // Remember, this is reversed!
                double x = gamepad1.left_stick_x*1.1;
                double rx = gamepad1.right_stick_x;

                this.driveTrain.drive(y, x, rx);
                idle();




            }

        }
    }

