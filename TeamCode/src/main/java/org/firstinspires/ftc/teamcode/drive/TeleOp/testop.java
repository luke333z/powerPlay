package org.firstinspires.ftc.teamcode.drive.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.LinkedList;
import java.util.Queue;

@TeleOp(group = "Driving")
public class testop extends LinearOpMode {

    SampleMecanumDrive mecanumDrive;
    double suppress1;
    int pp = 0, cp1 = 0, cp2 = 0;
    double suppressRotate;

    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        initialization();
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            run();
        }
    }

    private void initialization() {

        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        mecanumDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
        mecanumDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mecanumDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void controlWheels() {
        Pose2d poseEstimate = mecanumDrive.getPoseEstimate();
        Vector2d input = new Vector2d(
                -gamepad1.left_stick_y * suppress1,
                -gamepad1.left_stick_x * suppress1
        ).rotated(-poseEstimate.getHeading());
        mecanumDrive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -(gamepad1.right_trigger - gamepad1.left_trigger) * suppressRotate
                )
        );
    }
    private void suppressWheels() {
        if (gamepad1.right_bumper) {
            suppress1 = 0.5f;
            suppressRotate = 0.5f;
        } else {
            suppress1 = 1f;
            suppressRotate = 1f;
        }
    }

    private void run() {
        suppressWheels();
        controlWheels();
        debugTelemetry();
    }




    private void debugTelemetry() {
        telemetry.addData("position", mecanumDrive.getPoseEstimate());
        telemetry.update();
    }
}
