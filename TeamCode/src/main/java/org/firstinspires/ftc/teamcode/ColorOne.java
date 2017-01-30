/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cAddr;

@Autonomous(name = "Color One", group = "Sensor")
public class ColorOne extends LinearOpMode {

    ColorSensor colorSensorR;
	ColorSensor colorSensorL;
	ColorSensor colorSensorF; // Hardware Device Object
	DeviceInterfaceModule cdim;

	@Override
	public void runOpMode() throws InterruptedException {
		cdim = hardwareMap.deviceInterfaceModule.get("dim");

				// hsvValues is an array that will hold the hue, saturation, and value information.
				float hsvValues[] = {0F,0F,0F};

				// values is a reference to the hsvValues array.
				final float values[] = hsvValues;

				boolean bLedOn = true;


				// get a reference to our ColorSensor object.
//		colorSensorR = hardwareMap.colorSensor.get("color sensor right");
//		colorSensorR.setI2cAddress(I2cAddr.create8bit(0x5c));
//		colorSensorR.enableLed(true);

//		colorSensorL = hardwareMap.colorSensor.get("color sensor left");
//		//colorSensorL.setI2cAddress(I2cAddr.create8bit(0x4c));
//		colorSensorL.enableLed(true);
//
		colorSensorF = hardwareMap.colorSensor.get("color sensor beacon");
		colorSensorF.setI2cAddress(I2cAddr.create8bit(0x4c));
		//colorSensorF.enableLed(true);

				// Set the LED in the beginning

				// wait for the start button to be pressed.
				waitForStart();

				// while the op mode is active, loop and read the RGB data.
				// Note we use opModeIsActive() as our loop condition because it is an interruptible method.
				while (opModeIsActive()) {

					// check the status of the x button on either gamepad.

					// convert the RGB values to HSV values.

//					// send the info back to driver station using telemetry function.
//					telemetry.addData("right", 1);
//					telemetry.addData("LED", bLedOn ? "On" : "Off");
//					telemetry.addData("Clear", colorSensorR.alpha());
//					telemetry.addData("Red  ", colorSensorR.red());
//					//telemetry.addData("Green", colorSensor.green());
//					telemetry.addData("Blue ", colorSensorR.blue());
//					//telemetry.addData("Hue", hsvValues[0]);
//
//					telemetry.addData("left", 1);
//					telemetry.addData("LED", bLedOn ? "On" : "Off");
//					telemetry.addData("Clear", colorSensorL.alpha());
//					telemetry.addData("Red  ", colorSensorL.red());
//					//telemetry.addData("Green", colorSensor.green());
//					telemetry.addData("Blue ", colorSensorL.blue());
//					//telemetry.addData("Hue", hsvValues[0]);


					telemetry.addData("front", 1);
					telemetry.addData("LED", bLedOn ? "On" : "Off");
					telemetry.addData("Clear", colorSensorF.alpha());
					telemetry.addData("Red  ", colorSensorF.red());
					//telemetry.addData("Green", colorSensor.green());
					telemetry.addData("Blue ", colorSensorF.blue());
					//telemetry.addData("Hue", hsvValues[0]);





					// change the background color to match the color detected by the RGB sensor.
					// pass a reference to the hue, saturation, and value array as an argument
					// to the HSVToColor method.

					telemetry.update();
					idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
				}
			}
		}


