package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="vedantautotesting2", group="Linear OpMode")
public class vedantautotesting2 extends LinearOpMode {
    
    private MecanumDrive drive;
    private static final Pose2d START_POSE = new Pose2d(0, 0, 0);
    
    @Override
    public void runOpMode() {
        // Initialize drive
        drive = new MecanumDrive(hardwareMap, START_POSE);
        
        // Build an action: drive forward 10 inches, then turn 90° CCW, then drive forward 5 inches
        Action forwardThenTurnThenForward = drive.actionBuilder(START_POSE)
            .forward(10) // 10 inches forward
            .turnTo(Math.toRadians(90)) // 90° counter-clockwise
            .forward(5) // 5 inches forward after turn
            .build();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Movement", "Forward 10 in, 90° CCW, then Forward 5 in");
        telemetry.update();
        
        waitForStart();
        
        if (opModeIsActive()) {
            telemetry.addData("Status", "Executing sequence: F10 -> Turn 90° CCW -> F5");
            telemetry.update();

            // Execute the forward-then-turn-then-forward action
            Actions.runBlocking(forwardThenTurnThenForward);

            telemetry.addData("Status", "Complete");
            telemetry.update();
        }
    }
}

