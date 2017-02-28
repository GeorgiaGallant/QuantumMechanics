package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by student on 10/6/16.
 */

@Autonomous(name="GyroTestYAS", group="RGB + Encoder")
@Disabled


public class GyroTester extends LinearOpMode{
    //Gyro g = new Gyro(this);
    private ElapsedTime     runtime = new ElapsedTime();
    DcMotor FL;
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    //Servo   servo;
    DeviceInterfaceModule dim;
    @Override
    public void runOpMode() throws InterruptedException {
        dim = hardwareMap.deviceInterfaceModule.get("dim");
        Gyro g = new Gyro(this);

        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        FL = hardwareMap.dcMotor.get("FL");
        BL = hardwareMap.dcMotor.get("BL");
        FR = hardwareMap.dcMotor.get("FR");
        BR = hardwareMap.dcMotor.get("BR");

        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();

        while(g.isCalibrating()){
            g.calibrate();
            idle();
        }
      //  g.stayStraight();
        while (opModeIsActive()){
            telemetry.addData("ANGLE: ", g.getAngle());
            telemetry.addData("HEADING: ", g.getHeading());
            telemetry.update();
            idle();
        }

        //g.turnTo(30);
    }
    public void encoderDrive(double speed,
                             double LFInches, double RFInches, double LBInches, double RBInches,
                             double timeoutS) throws InterruptedException {

        int newLFTarget;
        int newRFTarget;
        int newLBTarget;
        int newRBTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLFTarget = FL.getCurrentPosition() + (int)(LFInches * COUNTS_PER_INCH);
            newLBTarget = BL.getCurrentPosition() + (int)(LBInches * COUNTS_PER_INCH);
            newRFTarget = FR.getCurrentPosition() + (int)(RFInches * COUNTS_PER_INCH);
            newRBTarget = BR.getCurrentPosition() + (int)(RBInches * COUNTS_PER_INCH);

            // newRightTarget = robot.rightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            FL.setTargetPosition(newLFTarget);
            FR.setTargetPosition(newRFTarget);
            BL.setTargetPosition(newLBTarget);
            BR.setTargetPosition(newRBTarget);

            // Turn On RUN_TO_POSITION
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            FL.setPower(Math.abs(speed));
            FR.setPower(Math.abs(speed));
            BL.setPower(Math.abs(speed));
            BR.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy())) {
                idle();
            }

            // Stop all motion;
            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);

            // Turn off RUN_TO_POSITION
            FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
}
