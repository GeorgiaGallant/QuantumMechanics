package org.firstinspires.ftc.teamcode;
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
 * Created by student on 10/13/16.
 */
@Autonomous(name = "line Follower", group = "Sensor")

public class lineFollowing extends LinearOpMode {
    ColorSensor colorSensor;  // Hardware Device Object
    @Override
    public void runOpMode() throws InterruptedException {

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);

        // bPrevState and bCurrState represent the previous and current state of the button.
        boolean bPrevState = false;
        boolean bCurrState = false;

        // bLedOn represents the state of the LED.
        boolean bLedOn = true;

        double LPower = .2;
        double RPower = .2;
        DcMotor LFMotor;
        DcMotor RFMotor;
        DcMotor LBMotor;
        DcMotor RBMotor;

        LFMotor = hardwareMap.dcMotor.get("LFMotor");
        LBMotor = hardwareMap.dcMotor.get("LBMotor");
        RFMotor = hardwareMap.dcMotor.get("RFMotor");
        RBMotor = hardwareMap.dcMotor.get("RBMotor");

        // get a reference to our ColorSensor object.
        colorSensor = hardwareMap.colorSensor.get("color sensor");

        // turn the LED on in the beginning, just so user will know that the sensor is active.
        colorSensor.enableLed(bLedOn);

        // wait for the start button to be pressed.
        waitForStart();

        // loop and read the RGB data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive())  {



            // check the status of the x button on either gamepad.
            bCurrState = gamepad1.x;

            // check for button state transitions.
            if ((bCurrState == true) && (bCurrState != bPrevState))  {

                // button is transitioning to a pressed state.  Toggle LED.
                // on button press, enable the LED.
                bLedOn = !bLedOn;
                colorSensor.enableLed(bLedOn);
            }

            // update previous state variable.
            bPrevState = bCurrState;

            // convert the RGB values to HSV values.
            Color.RGBToHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), hsvValues);

            // send the info back to driver station using telemetry function.
            telemetry.addData("LED", bLedOn ? "On" : "Off");
            telemetry.addData("Clear", colorSensor.alpha());
            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue ", colorSensor.blue());
            telemetry.addData("Hue", hsvValues[0]);



            if(colorSensor.alpha() > 30){
                telemetry.addData("RIGHT", 0);
                LPower = .4;
                RPower = -.4;
            }
            else if(colorSensor.alpha() < 20){
                telemetry.addData("LEFT", 0);
                RPower = .4;
                LPower = -.4;
            }
            else{
                telemetry.addData("STRAIGHT", 0);
                LPower = .35;
                RPower = .35;
            }

            LFMotor.setPower(-LPower);
            RFMotor.setPower(-RPower);
            RBMotor.setPower(RPower);
            LBMotor.setPower(LPower);

            LFMotor.setPower(0);
            RFMotor.setPower(0);
            RBMotor.setPower(0);
            LBMotor.setPower(0);
            sleep(5);
            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });

            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }

}
