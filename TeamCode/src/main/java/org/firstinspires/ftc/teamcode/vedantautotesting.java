package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Vedant Auto Testing", group="Linear OpMode")
public class vedantautotesting extends LinearOpMode {
    
    private MecanumDrive drive;
    private static final Pose2d START_POSE = new Pose2d(0, 0, 0);
    
    @Override
    public void runOpMode() {
        // Initialize drive
        drive = new MecanumDrive(hardwareMap, START_POSE);
        
        // Build trajectory to move 2 inches forward, turn 90 degrees left, move 87 inches forward, turn 90 degrees right
        Action moveForward = drive.actionBuilder(START_POSE)
                .lineToY(2.0)  // Move 2 inches forward (positive Y)
                .turnTo(Math.toRadians(90))  // Turn 90 degrees left (counter-clockwise)
                .lineToY(89.0)  // Move 87 more inches forward (total Y = 89.0)
                .turnTo(Math.toRadians(0))  // Turn 90 degrees right (back to 0 degrees)
                .build();
        
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Movement", "2\" fwd + 90° left + 87\" fwd + 90° right");
        telemetry.update();
        
        waitForStart();
        
        if (opModeIsActive()) {
            telemetry.addData("Status", "Moving forward...");
            telemetry.update();
            
            // Execute the movement
            Actions.runBlocking(moveForward);
            
            telemetry.addData("Status", "Complete");
            telemetry.addData("Final Position", "Y = 2.0 inches");
            telemetry.update();
        }
    }
}

