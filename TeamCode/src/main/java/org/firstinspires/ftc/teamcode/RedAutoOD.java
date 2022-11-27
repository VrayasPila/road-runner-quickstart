package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;



import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name = "RedAutoOD")
public class RedAutoOD extends LinearOpMode
{
    DcMotor left;
    DcMotor right;
    DcMotor rightBack;
    DcMotor leftBack;

    DcMotor Arm1;
    CRServo Claw;

    BNO055IMU imu;
    Orientation angles;

    //SampleMecanumDrive drivetrain;

    public void runOpMode() {
        left = hardwareMap.dcMotor.get("lf");
        right = hardwareMap.dcMotor.get("rf");
        leftBack = hardwareMap.dcMotor.get("lr");
        rightBack = hardwareMap.dcMotor.get("rr");
        Arm1 = hardwareMap.dcMotor.get("SL1");


        //drivetrain = new SampleMecanumDrive(hardwareMap);

        imu = (BNO055IMU) hardwareMap.get("imu");
        Claw = hardwareMap.crservo.get("claw");


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        left.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set arm run mode
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Zero Power Behavior
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();

        if (opModeIsActive())
        {
            DriveForward(0.25,0);

        }


    }
    public void DriveForward(double power, float targetAngle)
    {
        telemetry.clearAll();
        telemetry.addLine("DPad Up");
        telemetry.update();

        ResetEncoders();

        right.setTargetPosition(24 * 45);
        left.setTargetPosition(24 * 45);
        leftBack.setTargetPosition(24 * 45);
        rightBack.setTargetPosition(24 * 45);

        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        right.setPower(power);
        left.setPower(power);
        rightBack.setPower(power);
        leftBack.setPower(power);

        while (right.isBusy() && leftBack.isBusy() && left.isBusy() && rightBack.isBusy()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

            if (angles.firstAngle < targetAngle) {
                rightBack.setPower(power + 0.05);
                right.setPower(power + 0.05);
                leftBack.setPower(power - 0.05);
                left.setPower(power - 0.05);
            } else if (angles.firstAngle > targetAngle) {
                rightBack.setPower(power - 0.05);
                right.setPower(power - 0.05);
                leftBack.setPower(power + 0.05);
                left.setPower(power + 0.05);
            } else {
                right.setPower(power);
                left.setPower(power);
                rightBack.setPower(power);
                leftBack.setPower(power);
            }

        }

        telemetry.addData("Correction done", angles.firstAngle);
        telemetry.update();

        Stop();

    }



    public void ResetEncoders()
    {
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set arm run mode
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void Stop()
    {
        left.setPower(0);
        leftBack.setPower(0);
        right.setPower(0);
        rightBack.setPower(0);

    }
}

