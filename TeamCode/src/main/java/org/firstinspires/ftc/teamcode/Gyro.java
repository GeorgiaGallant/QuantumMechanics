package org.firstinspires.ftc.teamcode;/* Copyright (c) 2015 Qualcomm Technologies Inc*/

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * @author Swarup and Devin
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
    private int tolerance = 5;
    private LinearOpMode opMode;
    private double motorPower = 0.5;
    private double angle = 0;

    public Gyro(LinearOpMode opMode){
        this.hardwareMap = opMode.hardwareMap;
        this.opMode = opMode;
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

    public void turnTo(int degree){
        // turn right
        while(gyro.getIntegratedZValue() > degree + tolerance) {
            LFMotor.setPower(motorPower);
            LBMotor.setPower(motorPower);
            RFMotor.setPower(motorPower);
            RBMotor.setPower(motorPower);
            resetMotorPower();
            try {
                opMode.idle();
            } catch (InterruptedException e) {
                resetMotorPower();
                e.printStackTrace();
            }
        }
        while(gyro.getIntegratedZValue() < degree - tolerance){
           // turn left
            LFMotor.setPower(-motorPower);
            LBMotor.setPower(-motorPower);
            RFMotor.setPower(-motorPower);
            RBMotor.setPower(-motorPower);
            resetMotorPower();
            try {
                opMode.idle();
            } catch (InterruptedException e) {
                resetMotorPower();
                e.printStackTrace();
            }
        }
        angle = gyro.getIntegratedZValue();
        resetMotorPower();
    }

    /**
     * This function moves the robot to stay within 5 degrees of the heading
     */
    public void stayStraight(){
        while(gyro.getIntegratedZValue() > angle+tolerance || gyro.getIntegratedZValue() < angle-tolerance) {
            double angleZ = gyro.getIntegratedZValue();
            if (angleZ > angle+tolerance) {
                opMode.telemetry.addData("TURNING RIGHT ➡️️", 0);
                opMode.telemetry.update();
                LFMotor.setPower(motorPower);
                LBMotor.setPower(motorPower);
                RFMotor.setPower(motorPower);
                RBMotor.setPower(motorPower);
            } else if (angleZ < angle-tolerance) {
                opMode.telemetry.addData("TURNING LEFT ⬅️", 0);
                opMode.telemetry.update();
                LFMotor.setPower(-motorPower);
                LBMotor.setPower(-motorPower);
                RFMotor.setPower(-motorPower);
                RBMotor.setPower(-motorPower);
            }
            try {
                opMode.idle();
            } catch (InterruptedException e) {
                resetMotorPower();
                e.printStackTrace();
            }
        }
        angle = gyro.getIntegratedZValue();
    }

    private void resetMotorPower(){
        LFMotor.setPower(0);
        LBMotor.setPower(0);
        RFMotor.setPower(0);
        RBMotor.setPower(0);
    }

    private void setupDevices(){
        LFMotor = hardwareMap.dcMotor.get("LFMotor");
        LBMotor = hardwareMap.dcMotor.get("LBMotor");
        RFMotor = hardwareMap.dcMotor.get("RFMotor");
        RBMotor = hardwareMap.dcMotor.get("RBMotor");
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
    }
}