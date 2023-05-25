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

import java.util.LinkedList;
import java.util.Queue;

@TeleOp(group = "Driving")
public class lukeTeleOp extends LinearOpMode {
    SampleMecanumDrive mecanumDrive;
    DcMotorEx pusherMotor;
    double suppress1;
    int pp = 0, cp1 = 0, cp2 = 0;
    double suppressRotate;
    int extins_pos = -355;
    int retras_pos = 10;

    enum Mode{
        EXTINS,
        RETRAS,
        MANUAL,
        IDLE,
    }

    Mode currentMode = Mode.IDLE;

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
        pusherMotor = hardwareMap.get(DcMotorEx.class, "pusherMotor");
        pusherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pusherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pusherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        mecanumDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
        mecanumDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        pusherControl();

        debugTelemetry();

    }
    private void pusherControl() {

//        switch(currentMode){
//            case IDLE:
//                pusherMotor.setPower(0);
//                pusherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//                if(gamepad2.y)
//                    currentMode = Mode.EXTINS;
//                if(gamepad2.b)
//                    currentMode = Mode.MANUAL;
//
//                break;
//
//            case RETRAS:
//                pusherMotor.setTargetPosition(retras_pos);
//                pusherMotor.setPower(1);
//                pusherMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                if(gamepad2.y)
//                    currentMode = Mode.EXTINS;
//                if(gamepad2.b)
//                    currentMode = Mode.MANUAL;
//
//                break;
//
//            case EXTINS:
//                pusherMotor.setTargetPosition(extins_pos);
//                pusherMotor.setPower(-1);
//                pusherMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                if(gamepad2.a)
//                    currentMode = Mode.RETRAS;
//                if(gamepad2.b)
//                    currentMode = Mode.MANUAL;
//
//                break;
//
//            case MANUAL:
//                pusherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//                if(gamepad2.left_trigger != 0)
//                    pusherMotor.setPower(1);
//                if(gamepad2.right_trigger != 0)
//                    pusherMotor.setPower(-1);
//                else pusherMotor.setPower(0);
//        }

        if (gamepad2.left_trigger != 0)
            pusherMotor.setPower(1);

        else if (pusherMotor.getCurrentPosition() >= -10)
            pusherMotor.setPower(0);

        if (gamepad2.right_trigger != 0)
            pusherMotor.setPower(-1);

        if (gamepad2.b) {
            pusherMotor.setPower(-1);
            sleep(350);
            pusherMotor.setPower(1);
            sleep(400);
            pusherMotor.setPower(0);
        }
    }
    private void debugTelemetry() {

        telemetry.addData("plateBusy1", pusherMotor.isBusy());
        telemetry.addData("platePower", pusherMotor.getPower());
        telemetry.addData("pusherMotor_position", pusherMotor.getCurrentPosition());
        telemetry.addData("currentMode", currentMode.toString());

        telemetry.update();
    }
    private class MoveTarget {
        private DcMotorEx motor;
        private int position;

        public MoveTarget(DcMotorEx motor, int position) {
            this.motor = motor;
            this.position = position;
        }

        public DcMotorEx getMotor() {
            return motor;
        }

        public int getPosition() {
            return position;
        }
    }

    boolean down = false;

    private void executeCurrentMoveTarget() {
        if (moveTargets.isEmpty()) return;

        lukeTeleOp.MoveTarget moveTarget = moveTargets.peek();
        DcMotorEx motor = moveTarget.getMotor();

        motor.setTargetPosition(moveTarget.getPosition());
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(1);


        if (runtime.milliseconds() > 200) {//800
            moveTargets.remove();
            runtime.reset();
        }
    }

    Queue<lukeTeleOp.MoveTarget> moveTargets = new LinkedList<>();

    private void resetTargets() {
        moveTargets.clear();
        runtime.reset();
    }

    private void controlArm() {
        executeCurrentMoveTarget();
        lukeTeleOp.MoveTarget currentTarget;
        if(gamepad2.right_trigger!=0) {
            resetTargets();
            down = false;
            currentTarget = new MoveTarget(pusherMotor, 100);
            moveTargets.add(currentTarget);

        }
        else if(gamepad2.left_trigger!=0) {
            resetTargets();
            down = false;
            currentTarget = new MoveTarget(pusherMotor, 0);
            moveTargets.add(currentTarget);
        }


    }

    private void relax() {
        if (moveTargets.isEmpty() && runtime.milliseconds() > 1000) {
            pusherMotor.setPower(0);
        }
        
    }
}


