package org.firstinspires.ftc.teamcode.drive.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm{

    final static double LOWERING_POWER = 0.5;

    final static int FEED           = -7;
    final static int BOTTOM         = -250;
    final static int MIDDLE         = -450;
    final static int TOP            = -710;
    final static int INVERSE_TOP    = -2520;
    final static int INVERSE_MIDDLE = -2810;
    final static int INVERSE_BOTTOM = -2980;
    final static int CAP_UP         = -2200;
    final static int CAP_DOWN       = -2450;
    final static int INVERSE_FEED   = -3350;

    private static int targetLevel = 0;

    HardwareMap hwMap;

    private DcMotor motor;

    public Arm() {}

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        motor = hwMap.get(DcMotor.class, "arm");
        motor.setTargetPosition(0);
        motor.setPower(0);
        //motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void lower() {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(LOWERING_POWER);
    }

    public void calibrate() {
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(1);
        //motor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void setLevel(int level) {
        targetLevel = level;
        switch (level) {
            case 0: {
                motor.setTargetPosition(FEED);
                break;
            } case 1: {
                motor.setTargetPosition(BOTTOM);
                break;
            } case 2: {
                motor.setTargetPosition(MIDDLE);
                break;
            } case 3: {
                motor.setTargetPosition(TOP);
                break;
            } case 4: {
                motor.setTargetPosition(INVERSE_TOP);
                break;
            } case 5: {
                motor.setTargetPosition(INVERSE_MIDDLE);
                break;
            } case 6: {
                motor.setTargetPosition(INVERSE_BOTTOM);
                break;
            } case 7: {
                motor.setTargetPosition(CAP_UP);
                break;
            } case 8: {
                motor.setTargetPosition(CAP_DOWN);
                break;
            } case 9: {
                motor.setTargetPosition(INVERSE_FEED);
                break;
            }

        }
    }

    public int getTargetLevel(){
        return targetLevel;
    }

    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    public boolean isBusy () {
        return motor.isBusy();
    }

}
