/*
Based on another program by Robert Atkinson. See README for full license.
*/
package org.firstinspires.ftc.teamcode;

import android.text.style.TtsSpan;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

//@TeleOp(name="Grete/", group="Linear Opmode")  // @Autonomous(...) is the other common choice

public class Integration_All extends LinearOpMode {

    /* Declare OpMode members. */

    DcMotor NOM = null;
    DcMotor FL = null;
    DcMotor BL = null;
    DcMotor FR = null;
    DcMotor BR = null;
    DcMotor Elevator = null;
    DcMotor Conveyor = null;
    DcMotor Launch = null;
    Servo buttonPusher = null;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        FL = hardwareMap.dcMotor.get("FL");
        BL = hardwareMap.dcMotor.get("BL");
        FR = hardwareMap.dcMotor.get("FR");
        BR = hardwareMap.dcMotor.get("BR");
        NOM = hardwareMap.dcMotor.get("NOM");
        NOM.setDirection(DcMotor.Direction.REVERSE);

        Elevator = hardwareMap.dcMotor.get("Elevator");
        Conveyor = hardwareMap.dcMotor.get("Conveyor");
        Launch = hardwareMap.dcMotor.get("Launch");
        buttonPusher = hardwareMap.servo.get("buttonPusher");
        buttonPusher.setPosition(0.65);



        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        // launcherMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        // BL.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
            if(gamepad2.left_bumper) {
                buttonPusher.setPosition(0.6);
            }
            else if(gamepad2.right_bumper) {
                buttonPusher.setPosition(0.27);

            }
            else if(gamepad2.y){
                buttonPusher.setPosition(0.65);
            }

            if(Math.abs(gamepad1.left_stick_y) > 0.5) {
                FL.setPower(-gamepad1.left_stick_y);
                FR.setPower(gamepad1.left_stick_y);
                BR.setPower(gamepad1.left_stick_y);
                BL.setPower(-gamepad1.left_stick_y);
            }
            else if(Math.abs(gamepad1.left_stick_x) > 0.5){
                FL.setPower(-gamepad1.left_stick_x);
                BL.setPower(-gamepad1.left_stick_x);
                FR.setPower(-gamepad1.left_stick_x);
                BR.setPower(-gamepad1.left_stick_x);
            }else{
                FL.setPower(0);
                FR.setPower(0);
                BR.setPower(0);
                BL.setPower(0);
            }

            if(gamepad1.right_bumper==true){
                NOM.setPower(.6);
                Elevator.setPower(.6);
            }
            else if(gamepad1.left_bumper ==true) {
                NOM.setPower(-.6);
                Elevator.setPower(-.6);
            }
            else{
                NOM.setPower(0);
                Elevator.setPower(0);
            }

            if (gamepad2.b == true) {
                Conveyor.setPower(-1);
            }
            else if(gamepad2.x) {
                Conveyor.setPower(1);

            }
//            else if (gamepad2.x==true){
//                Conveyor.setPower(.2);
//            }
            else{
                Conveyor.setPower(0);
            }

            if(gamepad2.right_trigger > 0.5){
                Launch.setPower(.6);
            }
            else{
                Launch.setPower(0);
            }

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}
