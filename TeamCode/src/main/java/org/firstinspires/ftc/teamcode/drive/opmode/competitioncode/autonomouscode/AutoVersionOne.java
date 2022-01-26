package org.firstinspires.ftc.teamcode.drive.opmode.competitioncode.autonomouscode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.hardware.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.drive.hardware.Robot;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(name = "First Auto")
public class AutoVersionOne extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();
        if (opModeIsActive()) {
            Robot robot = new Robot();   //Uses heavily modified untested hardware

            robot.init(hardwareMap);

            Pose2d startPose = new Pose2d(6.44, -60, Math.toRadians(90));

            robot.driveTrain.setPoseEstimate(startPose);


            Trajectory StartAuto = robot.driveTrain.trajectoryBuilder(startPose)
                    .splineToSplineHeading(new Pose2d(-6, -40, Math.toRadians(120)), Math.toRadians(110))
                    .build();

            Trajectory MoveToWarehouse = robot.driveTrain.trajectoryBuilder(StartAuto.end())
                    .splineToSplineHeading(new Pose2d(20,-64,Math.toRadians(180)), Math.toRadians(0))
                    .splineToSplineHeading(new Pose2d(50,-64,Math.toRadians(180)), Math.toRadians(0))
                    .build();

            Trajectory MoveToShippingHub = robot.driveTrain.trajectoryBuilder(MoveToWarehouse.end())
                    .splineToSplineHeading(new Pose2d(20,-64,Math.toRadians(180)), Math.toRadians(180))
                    .splineToSplineHeading(new Pose2d(-5,-40,Math.toRadians(180)), Math.toRadians(100))
                    .build();

            robot.driveTrain.followTrajectory(StartAuto);
            robot.driveTrain.followTrajectory(MoveToWarehouse);
            robot.driveTrain.followTrajectory(MoveToShippingHub);



        }

    }
}
