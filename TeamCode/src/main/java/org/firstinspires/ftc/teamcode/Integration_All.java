/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

@TeleOp(name="Grete/", group="Linear Opmode")  // @Autonomous(...) is the other common choice
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
                buttonPusher.setPosition(0.8);
            }
            else if(gamepad2.right_bumper) {
                buttonPusher.setPosition(0.1);
            }
            else if(gamepad2.x) {
                buttonPusher.setPosition(0.45);
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
                Conveyor.setPower(-.2);
            }
            else if (gamepad2.x==true){
                Conveyor.setPower(.2);
            }
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
