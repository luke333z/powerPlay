package org.firstinspires.ftc.teamcode.testandra;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(group = "autonom")
public class Test1 extends LinearOpMode {
    SampleMecanumDrive _mecanumDrive;
    TrajectorySequence _trajectorySequence;

    @Override
    public void runOpMode() throws InterruptedException {
        Init();
        waitForStart();

        Run();
    }

    private void Init()
    {
        _mecanumDrive = new SampleMecanumDrive(hardwareMap);

        BuildTrajectories();
    }

    private void BuildTrajectories()
    {
        _trajectorySequence = _mecanumDrive.trajectorySequenceBuilder(new Pose2d(0))
                .forward(30)
                .back(30)
                .build();
    }

    private void Run()
    {
        while (!isStopRequested())
        {
            _mecanumDrive.followTrajectorySequence(_trajectorySequence);
        }
    }
}
