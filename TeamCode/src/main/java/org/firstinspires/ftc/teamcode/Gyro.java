package org.firstinspires.ftc.teamcode;/* Copyright (c) 2015 Qualcomm Technologies Inc*/

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * The Gyro class contains general functions involving the gyro that can be instantiated in any other
 * class
 */
public class Gyro{

    private ModernRoboticsI2cGyro gyro;
    private DcMotor LFMotor;
    private DcMotor RFMotor;
    private DcMotor LBMotor;
    private DcMotor RBMotor;
    private HardwareMap hardwareMap;

    /**
     * @param hardwareMap
     */
    public Gyro(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        setupDevices();
    }

    /**
     * Gets the gyro's current heading
     * @return int representing the heading in angles
     */
    public int getHeading(){
        return gyro.getHeading();
    }

    /**
     * Calibrate the gyro
     */
    public void calibrate(){
        gyro.calibrate();
    }

    /**
     * Check if the gyro is done calibrating
     * @return
     */
    public boolean isCalibrating(){
        return gyro.isCalibrating();
    }

    public void turnBy(int degree){
        //counterclockwise is positive
        int angleZ = gyro.getIntegratedZValue();
        double motor_power = 1;
        if(angleZ < degree) {
            while (angleZ < degree) {
                LFMotor.setPower(-motor_power);
                LBMotor.setPower(-motor_power);
                RFMotor.setPower(-motor_power);
                RBMotor.setPower(-motor_power);
                try {
                    Thread.sleep(50);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
        else{
            while (angleZ > degree) {
                LFMotor.setPower(motor_power);
                LBMotor.setPower(motor_power);
                RFMotor.setPower(motor_power);
                RBMotor.setPower(motor_power);
                try {
                    Thread.sleep(50);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    }

    private void setupDevices(){
        LFMotor = hardwareMap.dcMotor.get("LFMotor");
        LBMotor = hardwareMap.dcMotor.get("LBMotor");
        RFMotor = hardwareMap.dcMotor.get("RFMotor");
        RBMotor = hardwareMap.dcMotor.get("RBMotor");
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
    }
}
