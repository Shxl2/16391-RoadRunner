package org.firstinspires.ftc.teamcode.drive.hardware;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private final double INTAKE_POWER = 1;
    private final double OUTTAKE_POWER = -.2;


    HardwareMap hwMap = null;
    ElapsedTime timer = new ElapsedTime();


    public Intake() {}

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        leftMotor = hwMap.get(DcMotor.class, "feeder left");
        rightMotor = hwMap.get(DcMotor.class, "feeder right");

        leftMotor.setPower(0);
        rightMotor.setPower(0);

        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void stop() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public void intake() {
        leftMotor.setPower(-INTAKE_POWER);
        rightMotor.setPower(-INTAKE_POWER);
    }

    public void outtake() {
        leftMotor.setPower(-OUTTAKE_POWER);
        rightMotor.setPower(-OUTTAKE_POWER);
    }


}
