package org.firstinspires.ftc.teamcode;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "Field Centric TeleOp", group = "TeleOp")
public class FieldCentricTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        Follower follower = Constants.create(hardwareMap);
        follower.setPose(Pose.zero());

        telemetry.addLine("Ready. Face the robot away from you before pressing START.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Options re-zeroes heading so "forward" on the stick is the way the robot faces now
            if (gamepad1.optionsWasPressed()) {
                follower.setHeading(0);
            }

            Pose pose = follower.pose();

            DrivePowers robotPowers = new DrivePowers(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            );
            follower.manual(ManualDrive.fieldCentric(robotPowers, pose.heading()));
            follower.update();

            pose = follower.pose();
            telemetry.addData("X (in)", "%.2f", pose.x());
            telemetry.addData("Y (in)", "%.2f", pose.y());
            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(pose.heading()));
            telemetry.update();
        }

        follower.stop();
    }
}
