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

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.I2cAddr;

@Autonomous(name = "Color One", group = "Sensor")
//@Disabled                            // Comment this out to add to the opmode list
public class ColorOne extends LinearOpMode {

	ColorSensor sensorRGB1;
	//ColorSensor sensorRGB2;
	DeviceInterfaceModule cdim;

	// we assume that the LED pin of the RGB sensor is connected to
	// digital port 5 (zero indexed).
	// static final int LED_CHANNEL = 5;

	@Override
	public void runOpMode() throws InterruptedException {

		// hsvValues is an array that will hold the hue, saturation, and value information.
		// float hsvValues[] = {0F,0F,0F};


		cdim = hardwareMap.deviceInterfaceModule.get("dim");

		// get a reference to our ColorSensor object.
		sensorRGB1 = hardwareMap.colorSensor.get("color sensor beacon");
		sensorRGB1.setI2cAddress(I2cAddr.create8bit(0x5c));
		sensorRGB1.enableLed(false);


		// sensorRGB2 = hardwareMap.colorSensor.get("color2");


		// wait for the start button to be pressed.
		waitForStart();

		// loop and read the RGB data.
		// Note we use opModeIsActive() as our loop condition because it is an interruptible method.
		while (opModeIsActive())  {


			// send the info back to driver station using telemetry function.


			//      telemetry.addData("Clear 2", sensorRGB2.alpha());
			//      telemetry.addData("Red  2", sensorRGB2.red());
			//      telemetry.addData("Green 2", sensorRGB2.green());
			//      telemetry.addData("Blue 2", sensorRGB2.blue());
			//telemetry.addData("Hue", hsvValues[0]);



			telemetry.addData("Clear 1", sensorRGB1.alpha());
			telemetry.addData("Red  1", sensorRGB1.red());
			telemetry.addData("Green 1", sensorRGB1.green());
			telemetry.addData("Blue 1", sensorRGB1.blue());
			telemetry.update();




			idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop help me
		}
	}
}


