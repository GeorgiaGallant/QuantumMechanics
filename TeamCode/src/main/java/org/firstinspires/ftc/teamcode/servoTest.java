package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by student on 10/7/16.
 */


//@Autonomous(name="ServoTest", group="RGB + Encoder")

public class servoTest extends LinearOpMode {
    Servo   servo;
    DeviceInterfaceModule cdim;
    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.servo.get("buttonPusher");
        servo.setPosition(170);


    }

}
