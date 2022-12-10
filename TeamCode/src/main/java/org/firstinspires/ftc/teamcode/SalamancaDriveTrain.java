package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SalamancaDriveTrain {

    private static BNO055IMU imu;
    private final Telemetry telemetry;
    private final double correction_factor;

    protected DcMotor front_left_wheel = null;
    protected DcMotor back_left_wheel = null;
    protected DcMotor back_right_wheel = null;
    protected DcMotor front_right_wheel = null;

    public SalamancaDriveTrain(HardwareMap hardwareMap, Telemetry telemetry, GustavoDriveTrain.DirectionEnum direction) {
        this.telemetry = telemetry;
        this.correction_factor = direction.getCorrection();
    }

    public void initialzeDriveMotors(HardwareMap hardwareMap) {
        front_left_wheel = hardwareMap.dcMotor.get("front_left_wheel");
        back_left_wheel = hardwareMap.dcMotor.get("back_left_wheel");
        front_right_wheel = hardwareMap.dcMotor.get("front_right_wheel");
        back_right_wheel = hardwareMap.dcMotor.get("back_right_wheel");

        front_left_wheel.setDirection(DcMotorSimple.Direction.FORWARD);
        back_left_wheel.setDirection(DcMotorSimple.Direction.FORWARD);
        front_right_wheel.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right_wheel.setDirection(DcMotorSimple.Direction.REVERSE);

        front_left_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        front_left_wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left_wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_right_wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right_wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left_wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left_wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right_wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right_wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);


    }

    public void drive (double y, double x, double rx){

        // Retrieve the IMU from the hardware map
        // BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);

        front_left_wheel.setPower(y + x + rx);
        front_right_wheel.setPower(y - x + rx);
        back_left_wheel.setPower(y - x - rx);
        back_right_wheel.setPower(y + x - rx);

        double botHeading = -imu.getAngularOrientation().firstAngle;
        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        front_left_wheel.setPower(frontLeftPower);
        front_right_wheel.setPower(frontRightPower);
        back_left_wheel.setPower(backLeftPower);
        back_right_wheel.setPower(backRightPower);

        telemetry.addData("Front Left", frontLeftPower);
        telemetry.addData("Back Left", backLeftPower);
        telemetry.addData("Back Right", backRightPower);
        telemetry.addData("Front Right", frontRightPower);
        telemetry.addData("GyroSensor", botHeading);

        telemetry.update();



    }




}
