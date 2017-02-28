
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.robocol.RobocolParsable;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Quantum Mechanics
 * FTC Team 6051
 * Blue Autonomous
 */

@Autonomous(name="Rosie Blue Autonomous 2/21", group="Rosie Auto")
@Disabled

public class RosieBlue extends LinearOpMode {
    public static final double lineThresh = 25.000;
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor LFMotor;
    DcMotor RFMotor;
    DcMotor LBMotor;
    DcMotor RBMotor;
    DcMotor NOM;
    DcMotor Elevator;
    DcMotor Conveyor;
    DcMotor Launch;

    static final double     COUNTS_PER_MOTOR_REV    = 1440.000 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.000 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.000 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     TURN_SPEED              = 0.500;

    Servo servo;
    ColorSensor colorSensorR;
    ColorSensor colorSensorF;
    DeviceInterfaceModule cdim;
    @Override
    public void runOpMode() throws InterruptedException {

        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;

        double LPower = 0.200;
        double RPower = 0.200;
        double Kp = .005;
        double errorR = 0.00;


        colorSensorR = hardwareMap.colorSensor.get("right sensor");
        colorSensorF = hardwareMap.colorSensor.get("color sensor beacon");

        colorSensorR.setI2cAddress(I2cAddr.create8bit(0x5c));
        colorSensorF.setI2cAddress(I2cAddr.create8bit(0x4c));

        colorSensorF.enableLed(false); //reflects off of beacon if on
        colorSensorR.enableLed(true);

        cdim = hardwareMap.deviceInterfaceModule.get("dim");

        LFMotor = hardwareMap.dcMotor.get("FR");
        LBMotor = hardwareMap.dcMotor.get("BR");
        RFMotor = hardwareMap.dcMotor.get("FL");
        RBMotor = hardwareMap.dcMotor.get("BL");
        NOM = hardwareMap.dcMotor.get("NOM");
        Elevator = hardwareMap.dcMotor.get("Elevator");
        Launch = hardwareMap.dcMotor.get("Launch");
        servo = hardwareMap.servo.get("buttonPusher");

        NOM.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        drive(0.000, 0.000);
        waitForStart();
//        drive(.200, .200);
//
//        while (colorSensorR.alpha() < 20) {
//            telemetry.update();
//            drive(.300,.300);
//            sleep(50);
//            idle();
//        }
//        drive(0,0);
//        drive(-.150,-.500);
//        sleep(750);

        drive(0, 0);

//        while (colorSensorR.alpha() < 30) {
//            drive(0.000,.400);
//            sleep(50);
//            idle();
//        }
//        drive(0,0);
//        sleep(999999999);
        while(colorSensorF.alpha() < 2 ) { //.alpha is an int

            telemetry.addData("Right: ", RPower);
            telemetry.addData("Left: ", LPower);
            telemetry.addData("bottom right: ", colorSensorR.alpha());

            errorR = (lineThresh - (double)colorSensorR.alpha()) * .010; //25 should be a variable eg "lineThreshold"
            telemetry.addData("errorR: ", errorR);


            if (Math.abs(RPower-LPower)<.100) {
                RPower = RPower * (1.000 + (Kp * errorR));
            }
           // drive(LPower, RPower);
            telemetry.addData("RPower: ", RPower);
            telemetry.update();
            idle();
        }

        drive(0.000, 0.000);
        sleep(50);

        }


    public void drive(double left, double right){

        LFMotor.setPower(-left);
        RFMotor.setPower(right);
        RBMotor.setPower(right);
        LBMotor.setPower(-left);

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




