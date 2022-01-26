package org.firstinspires.ftc.teamcode.drive.hardware;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.List;

public class Robot {
    private final ElapsedTime period = new ElapsedTime();
    /* Public OpMode members. */

    public Carousel carousel = new Carousel();
    public Arm arm = new Arm();
    public Intake intake = new Intake();

    public MecanumDriveTrain driveTrain = new MecanumDriveTrain();

    public SensorDigitalTouch intakeSensor = new SensorDigitalTouch();

    WebcamName webcamName;

    HardwareMap hwMap;

    public Robot() {}

    public final static class ArmPosition {
        public final static int FEED           = 0;
        public final static int BOTTOM         = 1;
        public final static int MIDDLE         = 2;
        public final static int TOP            = 3;
        public final static int INVERSE_TOP    = 4;
        public final static int INVERSE_MIDDLE = 5;
        public final static int INVERSE_BOTTOM = 6;
        public final static int CAP_UP         = 7;
        public final static int CAP_DOWN       = 8;
        public final static int INVERSE_FEED   = 9;
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        // Define and Initialize Motors


        //map motors
        intake.init(hwMap);
        arm.init(hwMap);
        driveTrain.init(hwMap);
        carousel.init(hwMap);
    }

    public void drive(double forward, double strafe, double turn) {
        Pose2d poseEstimate = driveTrain.getPoseEstimate();

        // Create a vector from the gamepad x/y inputs
        Vector2d input = new Vector2d(
                -forward,
                -strafe
        );

        // Pass in the rotated input + right stick value for rotation
        // Rotation is not part of the rotated input thus must be passed in separately
        driveTrain.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -turn
                )
        );
    }
<<<<<<< Updated upstream


=======
>>>>>>> Stashed changes
    public void displayTelemetry(Telemetry telemetry) {
        final List<Double> wheelLocations = driveTrain.getWheelPositions();
        telemetry.addLine("Wheel position")
                .addData("Front Left", -wheelLocations.get(0))
                .addData("Front Right", -wheelLocations.get(1))
                .addData("Back Left", -wheelLocations.get(2))
                .addData("Back Right", -wheelLocations.get(3));
        telemetry.addLine("Arm")
                .addData("Target Level", arm.getTargetLevel())
                .addData("Encoder Position", arm.getCurrentPosition());
        telemetry.update();
    }
}
