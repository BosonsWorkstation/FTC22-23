package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous (name = "Model Auto", group = "Linear Opmode")
public class ModelAuto extends LinearOpMode{

    public GustavoDriveTrain driveTrain;
    private static final GustavoDriveTrain.DirectionEnum direction = GustavoDriveTrain.DirectionEnum.WEST;

    @Override
    public void runOpMode() throws InterruptedException {

        this.driveTrain = new GustavoDriveTrain(this.hardwareMap, this.telemetry, direction);

        waitForStart();

        this.driveTrain.autoMove(200, 0.8, true);
        this.driveTrain.autoCrab(-100, 0.8, true);


    }
}
