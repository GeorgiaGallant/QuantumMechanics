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

@TeleOp(name="Rosie/", group="Linear Opmode")  // @Autonomous(...) is the other common choice
public class Rosie extends LinearOpMode {

    /* Declare OpMode members. */

    DcMotor NOM = null;
    DcMotor FL = null;
    DcMotor BL = null;
    DcMotor FR = null;
    DcMotor BR = null;
    DcMotor Elevator = null;
    DcMotor Launch = null;
    Servo door = null;
    Servo buttonPusher = null;
    DcMotor Lift = null;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        FL = hardwareMap.dcMotor.get("FL");
        BL = hardwareMap.dcMotor.get("BL");
        FR = hardwareMap.dcMotor.get("FR");
        BR = hardwareMap.dcMotor.get("BR");
        NOM = hardwareMap.dcMotor.get("NOM");
        Elevator = hardwareMap.dcMotor.get("Elevator");
        Launch = hardwareMap.dcMotor.get("Launch");
        Lift = hardwareMap.dcMotor.get("Lift");
        door = hardwareMap.servo.get("door");
        door.setPosition(.3 );//open
        buttonPusher = hardwareMap.servo.get("buttonPusher");
        buttonPusher.setPosition(.7);


        // Wait for the game to start (drive,,,,
                // p-r presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //driver gamepad 1

            //drive

            if(gamepad1.dpad_down==true) {
                FL.setPower(-.25);
                FR.setPower(.25);
                BR.setPower(.25);
                BL.setPower(-.25);
            }
            else if(gamepad1.dpad_up==true) {
                FL.setPower(.25);
                FR.setPower(-.25);
                BR.setPower(-.25);
                BL.setPower(.25);
            }
            else if(gamepad1.dpad_right==true) {
                FL.setPower(-.6);
                FR.setPower(-.6);
                BR.setPower(-.6);
                BL.setPower(-.6);
            }
            else if(gamepad1.dpad_left==true) {
                FL.setPower(.6);
                FR.setPower(.6);
                BR.setPower(.6);
                BL.setPower(.6);
            }
            else if(Math.abs(gamepad1.left_stick_y) > 0.5) {
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
            }

            else{
                FL.setPower(0);
                FR.setPower(0);
                BR.setPower(0);
                BL.setPower(0);
            }
            //pick up

            if(gamepad1.right_bumper==true){
                NOM.setPower(-.6);
                Elevator.setPower(-.6);
            }
            else if(gamepad1.left_bumper ==true) {
                NOM.setPower(.6);
                Elevator.setPower(.6);
            }
            else{
                NOM.setPower(0);
                Elevator.setPower(0);
            }
            // butto pusher

            if(gamepad1.b == true || gamepad2.b == true){
                buttonPusher.setPosition(.7); //open
            }
            else if(gamepad1.x == true || gamepad2.x == true){
                buttonPusher.setPosition(0); //open
            }

            //gunner game pad 2

            //launch
            if(gamepad2.right_trigger > 0.5){
                Launch.setPower(.6);
            }
            else{
                Launch.setPower(0);
            }

            //lift

            if(Math.abs(gamepad2.left_stick_y) > 0.3) {
                Lift.setPower(gamepad2.left_stick_y);
            }
            else{
                Lift.setPower(0);
            }

            //door
            if(gamepad2.left_bumper == true){
               door.setPosition(.85); //open
            }
            else{
                door.setPosition(.3); //closed
            }

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}

