package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by student on 10/6/16.
 */
@Autonomous(name="RGB + Encoder", group="RGB + Encoder")
@Disabled

public class Encoder_RGB_Integration extends LinearOpMode{
    private ElapsedTime     runtime = new ElapsedTime();
    DcMotor LFMotor;
    DcMotor RFMotor;
    DcMotor LBMotor;
    DcMotor RBMotor;
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    Servo   servo;
    ColorSensor sensorRGB1;
    //ColorSensor sensorRGB2;
    DeviceInterfaceModule cdim;
    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        // Send telemetry message to signify robot waiting;
        cdim = hardwareMap.deviceInterfaceModule.get("dim");


        // get a reference to our ColorSensor object.
        sensorRGB1 = hardwareMap.colorSensor.get("color1");

        //Resetting encoders
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        LFMotor = hardwareMap.dcMotor.get("LFMotor");
        LBMotor = hardwareMap.dcMotor.get("LBMotor");
        RFMotor = hardwareMap.dcMotor.get("RFMotor");
        RBMotor = hardwareMap.dcMotor.get("RBMotor");

        servo = hardwareMap.servo.get("buttonPusher");

        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                RFMotor.getCurrentPosition(),
                RBMotor.getCurrentPosition(),
                RBMotor.getCurrentPosition(),
                LFMotor.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,verse movement is obtained by s
        // Note: Reetting a negative distance (not speed)
        encoderDrive(DRIVE_SPEED,  9,-9,9, -9, 5.0);  //drive forward
        encoderDrive(TURN_SPEED,   4, 4, 4, 4, 4.0);   // turn right
        encoderDrive(DRIVE_SPEED,  19.3,-19.3,19.3, -19.3, 5.0);  //drive forward
        encoderDrive(TURN_SPEED,   2.35, 2.35, 2.35, 2.35, 4.0);// S2: Turn Right 12 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED,  3.0,-3.0,3.0, -3.0, 5.0);  //drive forward



        telemetry.addData("Path", "Complete");
        telemetry.update();
        int red = 0;
        int blue = 0;

       // while(opModeIsActive()) {
            telemetry.addData("Clear 1", sensorRGB1.alpha());
                telemetry.addData("Red  1", sensorRGB1.red());
                telemetry.addData("Green 1", sensorRGB1.green());
                telemetry.addData("Blue 1", sensorRGB1.blue());
                if(sensorRGB1.red()>sensorRGB1.blue()){
                    telemetry.addData("COLOR: ", "red");
                    servo.setPosition(170);
                    sleep(100);

                    // red++;
                }
                else {
                    telemetry.addData("COLOR: ", "blue");
                    servo.setPosition(-170);
                    sleep(100);
                   // blue++;
                }
                telemetry.update();
            idle();
       // }
//        if(red > blue){
//            servo.setPosition(170);
//        }
//        if(blue >  red){
//            servo.setPosition(170);
//        }
//        while(opModeIsActive()){
//            telemetry.addData("Red: ", red);
//            telemetry.addData("Blue: ", blue);
//        }
//        idle();

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
            newLFTarget = LFMotor.getCurrentPosition() + (int)(LFInches * COUNTS_PER_INCH);
            newLBTarget = LBMotor.getCurrentPosition() + (int)(LBInches * COUNTS_PER_INCH);
            newRFTarget = RFMotor.getCurrentPosition() + (int)(RFInches * COUNTS_PER_INCH);
            newRBTarget = RBMotor.getCurrentPosition() + (int)(RBInches * COUNTS_PER_INCH);

            // newRightTarget = robot.rightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            LFMotor.setTargetPosition(newLFTarget);
            RFMotor.setTargetPosition(newRFTarget);
            LBMotor.setTargetPosition(newLBTarget);
            RBMotor.setTargetPosition(newRBTarget);

            // Turn On RUN_TO_POSITION
            LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            LFMotor.setPower(Math.abs(speed));
            RFMotor.setPower(Math.abs(speed));
            LBMotor.setPower(Math.abs(speed));
            RBMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (LFMotor.isBusy() && RFMotor.isBusy() && LBMotor.isBusy() && RBMotor.isBusy())) {

                // Display it for the driver.
//                telemetry.addData("Path1",  "Running to %7d :%7d", newLFTarget,  newRFTarget, newLBTarget, newRBTarget);
//                telemetry.addData("Path2",  "Running at %7d :%7d",
//                        LFMotor.getCurrentPosition(),
//                        RFMotor.getCurrentPosition(),
//                        LBMotor.getCurrentPosition(),
//                        RBMotor.getCurrentPosition());
                //telemetry.update();
//
//                telemetry.addData("Clear 1", sensorRGB1.alpha());
//                telemetry.addData("Red  1", sensorRGB1.red());
//                telemetry.addData("Green 1", sensorRGB1.green());
//                telemetry.addData("Blue 1", sensorRGB1.blue());
//                if(sensorRGB1.red()>sensorRGB1.blue()){
//                    telemetry.addData("COLOR: ", "red");
//                }
//                else {
//                    telemetry.addData("COLOR: ", "blue");
//                }
//                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            LFMotor.setPower(0);
            RFMotor.setPower(0);
            LBMotor.setPower(0);
            RBMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}
