package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.ElapsedTime;
import android.app.Activity;
import android.graphics.Color;
import android.graphics.Path;
import android.view.View;
import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.Servo;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Quantum Mechanics
 * FTC Team 6051
 * Blue Autonomous
 */
@Autonomous(name="Blue Autonomous FINAL", group="RGB + Encoder")
@Disabled
public class Blue_Autonomous_Final extends LinearOpMode{
    private ElapsedTime     runtime = new ElapsedTime();
    DcMotor LFMotor;
    DcMotor RFMotor;
    DcMotor LBMotor;
    DcMotor RBMotor;
    DcMotor NOM;
    DcMotor Elevator;
    DcMotor Conveyor;
    DcMotor Launch;

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED_FAST             = 0.5;
    static final double     DRIVE_SPEED_SLOW             = 0.5;
    static final double     TURN_SPEED              = 0.5;

    Servo   servo;
    //ColorSensor sensorRGB1;
    //ColorSensor colorSensorL;
    ColorSensor colorSensorR;// Hardware Device Object
    ColorSensor colorSensorF;
    ColorSensor colorSensorL;
    //ColorSensor sensorRGB2;
    DeviceInterfaceModule cdim;
    @Override
    public void runOpMode() throws InterruptedException {

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;


        double LPower = 0.2;
        double RPower = 0.2;
        double Kp = (1 - LPower) / (15);
        double errorR = 0.0;
        double errorL = 0.0;


        // get a reference to our ColorSensor object.
    //    colorSensorL = hardwareMap.colorSensor.get("color sensor left");
        colorSensorR = hardwareMap.colorSensor.get("color sensor right");
        colorSensorF = hardwareMap.colorSensor.get("color sensor beacon");
        colorSensorL = hardwareMap.colorSensor.get("color sensor left");


        colorSensorR.setI2cAddress(I2cAddr.create8bit(0x5c));
        colorSensorF.setI2cAddress(I2cAddr.create8bit(0x4c));



        colorSensorF.enableLed(false);
        colorSensorR.enableLed(true);
        colorSensorL.enableLed(true);


        // turn the LED on in the beginning, just so user will know that the sensor is active.

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        // Send telemetry message to signify robot waiting;
        cdim = hardwareMap.deviceInterfaceModule.get("dim");


        // get a reference to our ColorSensor object.
        //sensorRGB1 = hardwareMap.colorSensor.get("beaconColor");

        //Resetting encoders
        LFMotor = hardwareMap.dcMotor.get("FL");
        LBMotor = hardwareMap.dcMotor.get("BL");
        RFMotor = hardwareMap.dcMotor.get("FR");
        RBMotor = hardwareMap.dcMotor.get("BR");
        NOM = hardwareMap.dcMotor.get("NOM");
        Elevator = hardwareMap.dcMotor.get("Elevator");
        Conveyor = hardwareMap.dcMotor.get("Conveyor");
        Launch = hardwareMap.dcMotor.get("Launch");

        NOM.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        servo = hardwareMap.servo.get("buttonPusher");

        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                RFMotor.getCurrentPosition(),
                RBMotor.getCurrentPosition(),
                RBMotor.getCurrentPosition(),
                LFMotor.getCurrentPosition());
        telemetry.update();
        //servo.setPosition(0.65);



        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,verse movement is obtained by s
        // Note: Reetting a negative distance (not sped)
   //     telemetry.addData("STARTED", 6666);
        //encoderDrive(DRIVE_SPEED_FAST, -9, 9, -9, 9, 5.0);  //drive forward
        LFMotor.setPower(0);
        LBMotor.setPower(0);
        RFMotor.setPower(0);
        RBMotor.setPower(0);
        sleep(7000);




        /*while((getRuntime() - start)< 2){
            Launch.setPower(.6);
            Conveyor.setPower(1);
        }*/
        Launch.setPower(0);
        //servo.setPosition(.6);


        encoderDrive(DRIVE_SPEED_FAST, -14.5, 14.5, -14.5, 14.5, 12.0);
        sleep(50);
        LFMotor.setPower(0);
        LBMotor.setPower(0);
        RFMotor.setPower(0);
        RBMotor.setPower(0);
        sleep(1500);

        encoderDrive(TURN_SPEED, 4.0, 4.0, 4.0, 4.0, 4.0);
        sleep(1500);

        double start = getRuntime();

        while ((getRuntime() - start) < 2) {
            Launch.setPower(.6);
            idle();
        }

        Launch.setPower(0);
        start = getRuntime();
        while ((getRuntime() - start) < 2) {
            Conveyor.setPower(-1);
            idle();
        }

        start = getRuntime();
        while ((getRuntime() - start) < 2.2) {
            Launch.setPower(.6);
            idle();
        }
        Conveyor.setPower(0);
        Launch.setPower(0);

        encoderDrive(TURN_SPEED, -.5, -.5, -.5, -.5, 3.0);


        encoderDrive(DRIVE_SPEED_FAST, -9.5, 9.5, -9.5, 9.5, 10);






//        while (colorSensorR.alpha() < 20) {
//           //telemetry.addData("color", colorSensorL.alpha());
//            //telemetry.addData("color", colorSensorR.alpha());
//            telemetry.update();
//            LFMotor.setPower(-.4);
//            LBMotor.setPower(-.4);
//            RFMotor.setPower(.4);
//            RBMotor.setPower(.4);
//            sleep(50);
//            idle();
//        }
//
//        LFMotor.setPower(0);
//        LBMotor.setPower(0);
//        RFMotor.setPower(0);
//        RBMotor.setPower(0);
//        sleep(200);

        //encoderDrive(TURN_SPEED, -3.5, -3.5, -3.5, -3.5, 4.0);   // turn right

//        while (colorSensorR.alpha() < 20) {
//            telemetry.update();
//            LFMotor.setPower(-.4);
//            LBMotor.setPower(-.4);
//            RFMotor.setPower(-.4);
//            RBMotor.setPower(-.4);
//            sleep(50);
//            LFMotor.setPower(0);
//            LBMotor.setPower(0);
//            RFMotor.setPower(0);
//            RBMotor.setPower(0);
//            sleep(50);
//            idle();
//        }

//       encoderDrive(TURN_SPEED, -3, -3, -3, -3, 5.0);
//        sleep(200);


//        while (colorSensorR.alpha() < 30) {
//            telemetry.update();
//            LFMotor.setPower(.4);
//            LBMotor.setPower(.4);
//            RFMotor.setPower(.6);
//            RBMotor.setPower(.6);
//            sleep(100);
//            LFMotor.setPower(0);
//            LBMotor.setPower(0);
//            RFMotor.setPower(0);
//            RBMotor.setPower(0);
//            sleep(50);
//            idle();
//        }
//
//
//        // NOW WE SHOULD DO LINE FOLLOWING
//        LFMotor.setPower(0);
//        LBMotor.setPower(0);
//        RFMotor.setPower(0);
//        RBMotor.setPower(0);
//        sleep(50);
//
//        while(colorSensorF.alpha() < 2 ) {
//            telemetry.addData("ERROR R: ", errorR);
//            telemetry.addData("front", colorSensorF.alpha());
//            telemetry.update();
//            LPower = .2;
//            RPower = .2;
//
//            errorR = Math.abs(colorSensorR.alpha() - 25);
//            if (colorSensorR.alpha() > 30) {
//                LPower = LPower * (1 + (Kp * errorR));
//                RPower = RPower * (1 - (Kp * errorR));
//
//            }
//             else if (colorSensorR.alpha() < 20) {
//                LPower = .2;
//                RPower = .4;
//            }
//            else {
//               RPower = .2;
//                LPower = .4;
//            }
//
//            LFMotor.setPower(-LPower);
//            RFMotor.setPower(RPower);
//            RBMotor.setPower(RPower);
//            LBMotor.setPower(-LPower);
//            sleep(100);
//            LFMotor.setPower(0);
//            RFMotor.setPower(0);
//            RBMotor.setPower(0);
//            LBMotor.setPower(0);
//            sleep(50);
//            idle();
//
//        }
//
//            LFMotor.setPower(0);
//            RFMotor.setPower(0);
//            RBMotor.setPower(0);
//            LBMotor.setPower(0);
//            sleep(50);
//
//
//            encoderDrive(TURN_SPEED, .7, .7, .7, .7, 3.0);
//
//
//            idle();
//            int red =0;
//            int blue = 0;
//
//        for(int x =0; x<100; x++) {
//            if (colorSensorF.red() > colorSensorF.blue()) {
//                red++;
//            } else {
//                blue++;
//            }
//        }
//
//        LFMotor.setPower(0);
//        RFMotor.setPower(0);
//        RBMotor.setPower(0);
//        LBMotor.setPower(0);
//        sleep(200);
//
//        telemetry.addData("red",  red);
//        telemetry.addData("bLue", blue);
//        telemetry.update();
//        if(blue>red){
//            telemetry.addData("COLOR: ", "blue");
//            servo.setPosition(.63);
//        }
//        else {
//            telemetry.addData("COLOR: ", "red");
//            //servo.setPosition(.24);
//            encoderDrive(TURN_SPEED, 0.3, 0.3, 0.3, 0.3, 4.0);
//            sleep(50);
//        }
//        LFMotor.setPower(0);
//        RFMotor.setPower(0);
//        RBMotor.setPower(0);
//        LBMotor.setPower(0);
//        sleep(1000);
//
//        encoderDrive(TURN_SPEED, -.7, -.7, -.7, -.7, 3.0);
//
//
//        long timer = System.currentTimeMillis();
//        while(System.currentTimeMillis() - timer < 800) {
//            telemetry.addData("ERROR R: ", errorR);
//            telemetry.addData("front", colorSensorF.alpha());
//            telemetry.update();
//            LPower = .2;
//            RPower = .2;
//
//            errorR = Math.abs(colorSensorR.alpha() - 25);
//            if (colorSensorR.alpha() > 30) {
//                LPower = LPower * (1 + (Kp * errorR));
//                RPower = RPower * (1 - (Kp * errorR));
//
//            }
//            else if (colorSensorR.alpha() < 20) {
//                LPower = .2;
//                RPower = .4;
//            }
//            else {
//                RPower = .2;
//                LPower = .2;
//            }
//
//            LFMotor.setPower(-LPower);
//            RFMotor.setPower(RPower);
//            RBMotor.setPower(RPower);
//            LBMotor.setPower(-LPower);
//            sleep(50);
//            idle();
//        }
//
//        LFMotor.setPower(0);
//        RFMotor.setPower(0);
//        RBMotor.setPower(0);
//        LBMotor.setPower(0);
//        sleep(50);
//
//        encoderDrive(DRIVE_SPEED_FAST, 4, -4, 4, -4, 5.0);
//        sleep(100);
//
//        encoderDrive(TURN_SPEED, 0.3, 0.3, 0.3, 0.3, 4.0);
//        sleep(50);
//
//        if(blue<red){
//            encoderDrive(TURN_SPEED, 0.45, 0.45, 0.45, 0.45, 4.0);
//            sleep(50);
//        }
//
//        encoderDrive(DRIVE_SPEED_FAST, -4.2, 4.2, -4.2, 4.2, 5.0);
//        sleep(100);
//
//        encoderDrive(DRIVE_SPEED_FAST, 21, -21, 21, -21, 5.0);
//        sleep(200);


//
//        encoderDrive(DRIVE_SPEED_SLOW, -13, 13, -13, 13, 5.0);
//        encoderDrive(TURN_SPEED, 13, 13, 13, 13, 4.0);   // turn right
//        encoderDrive(DRIVE_SPEED_SLOW, 2, -2, 2, -2, 5.0);
//


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