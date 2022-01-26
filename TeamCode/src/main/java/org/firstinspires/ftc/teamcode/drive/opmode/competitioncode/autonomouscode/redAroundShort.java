package org.firstinspires.ftc.teamcode.drive.opmode.competitioncode.autonomouscode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.hardware.Robot;

@Autonomous(name = "Red Auto: Warehouse - Around - Right - Short", group = "Red")
//@Disabled
public class redAroundShort extends AutonomousTemplate{
    Robot robot = new Robot();
    private int autoState = 1;
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void setUp() {
        super.setUp();
        robot.init(hardwareMap);
        robot.arm.lower();
    }

    public void startUp(){
        super.startUp();
        robot.arm.calibrate();
    }

    @Override
    public void left() {
        super.left();
        robot.arm.setLevel(Robot.ArmPosition.BOTTOM);
    }
    @Override
    public void right() {
        super.right();
        robot.arm.setLevel(Robot.ArmPosition.MIDDLE);

    }
    @Override
    public void unseen() {
        super.unseen();
        robot.arm.setLevel(Robot.ArmPosition.TOP);

    }

    @Override
    public void regularAutonomous() {
        super.regularAutonomous();
        Pose2d startPose = new Pose2d(6.44, -63, Math.toRadians(90));

        robot.driveTrain.setPoseEstimate(startPose);


        Trajectory startAuto = robot.driveTrain.trajectoryBuilder(startPose)
                .splineToSplineHeading(new Pose2d(-5, -45, Math.toRadians(110)), Math.toRadians(120))
                .build();

        Trajectory moveToWarehouse = robot.driveTrain.trajectoryBuilder(startAuto.end(), Math.toRadians(280))
                .splineToSplineHeading(new Pose2d(20,-66,Math.toRadians(180)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(50,-66,Math.toRadians(180)), Math.toRadians(0))
                .build();

        Trajectory moveToShippingHub = robot.driveTrain.trajectoryBuilder(moveToWarehouse.end(),Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(20,-66,Math.toRadians(180)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-5,-40,Math.toRadians(110)), Math.toRadians(100))
                .build();

        while (opModeIsActive()) {
            switch(autoState) {
                case 1: {
                    robot.driveTrain.followTrajectoryAsync(startAuto);
                    autoState = 3;
                    runtime.reset();
                    break;
                }
                case 2: {
                    robot.intake.outtake();
                    if (!robot.driveTrain.isBusy()) {
                        robot.driveTrain.followTrajectoryAsync(moveToWarehouse);
                        robot.intake.intake();
                        autoState = 4;
                    }
                }
                case 3: {
                    robot.driveTrain.update();
                    if (runtime.time() > 1) {
                        autoState = 2;
                    }
                    break;
                }
                case 4: {
                    robot.driveTrain.update();
                    if (runtime.time() > 2) {
                        break;
                    }
                    break;
                }

            }
        }
        while(opModeIsActive() && robot.driveTrain.isBusy()) {
            robot.driveTrain.update();
        }
        //robot.intake.outtake();
        robot.driveTrain.followTrajectoryAsync(moveToWarehouse);
        //sleep(1000);
        //robot.intake.intake();
        while(opModeIsActive() && robot.driveTrain.isBusy()) {
            robot.driveTrain.update();
        }
        robot.driveTrain.followTrajectory(moveToShippingHub);
    }
}
