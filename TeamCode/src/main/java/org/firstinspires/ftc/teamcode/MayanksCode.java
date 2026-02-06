package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Autonomous(name="Mayanks Code", group="Linear OpMode")
public class MayanksCode extends LinearOpMode {
    
    private MecanumDrive drive;
    private DcMotor intake;
    private DcMotor conveyor;
    private DcMotor indexer;
    private DcMotor shooter;
    private Servo trigger;
    private TouchSensor indexerSensor;
    private static final Pose2d START_POSE = new Pose2d(0, 0, 0);
    
    @Override
    public void runOpMode() {
        // Initialize drive
        drive = new MecanumDrive(hardwareMap, START_POSE);
        
        // Initialize intake (hardware name: "intake")
        try {
            intake = hardwareMap.get(DcMotor.class, "intake");
        } catch (Exception e) {
            intake = null;
        }
        // Initialize conveyor (hardware name: "conveyor")
        try {
            conveyor = hardwareMap.get(DcMotor.class, "conveyor");
        } catch (Exception e) {
            conveyor = null;
        }
        // Initialize indexer (hardware name: "indexer")
        try {
            indexer = hardwareMap.get(DcMotor.class, "indexer");
        } catch (Exception e) {
            indexer = null;
        }
        // Initialize indexer sensor (hardware name: "indexerSensor")
        try {
            indexerSensor = hardwareMap.get(TouchSensor.class, "indexerSensor");
        } catch (Exception e) {
            indexerSensor = null;
        }
        // Initialize shooter (hardware name: "shooter")
        try {
            shooter = hardwareMap.get(DcMotor.class, "shooter");
        } catch (Exception e) {
            shooter = null;
        }
        // Initialize trigger servo (hardware name: "trigger")
        try {
            trigger = hardwareMap.get(Servo.class, "trigger");
            trigger.setPosition(0.0); // Set to initial position (up)
        } catch (Exception e) {
            trigger = null;
        }

        // Build an action: drive forward 10 inches, then turn 90° CW, then drive backward 5 inches, then drive forward 5 inches, then turn 90° CCW, then drive backward 10 inches, then turn 90° CW, then drive forward 10 inches, then turn 90° CCW
        Action complexSequence = drive.actionBuilder(START_POSE)
            .forward(10) // 10 inches forward
            .turnTo(Math.toRadians(-90)) // 90° clockwise
            .back(5) // 5 inches backward after turn
            .forward(5) // 5 inches forward
            .turnTo(Math.toRadians(90)) // 90° counter-clockwise
            .back(10) // 10 inches backward
            .turnTo(Math.toRadians(0)) // 90° clockwise (back to 0°)
            .forward(10) // 10 inches forward
            .turnTo(Math.toRadians(90)) // 90° counter-clockwise
            .build();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Movement", "Forward 10 in, 90° CW, Backward 5 in, Forward 5 in, 90° CCW, Backward 10 in, 90° CW, Forward 10 in, 90° CCW");
        telemetry.update();
        
        waitForStart();
        
        if (opModeIsActive()) {
            telemetry.addData("Status", "Executing sequence: F10 -> Turn 90° CW -> B5 -> F5 -> Turn 90° CCW -> B10 -> Turn 90° CW -> F10 -> Turn 90° CCW");
            telemetry.update();

            // Execute the complex sequence
            Actions.runBlocking(complexSequence);

            // Start intake, conveyor, indexer, and shooter after driving 5 inches forward
            if (intake != null) {
                intake.setPower(1.0);
            }
            if (conveyor != null) {
                conveyor.setPower(1.0);
            }
            if (indexer != null) {
                indexer.setPower(1.0);
            }
            if (intake == null && conveyor == null && indexer == null) {
                telemetry.addData("Intake/Conveyor/Indexer", "Not configured (names: 'intake','conveyor','indexer')");
            } else {
                telemetry.addData("Intake/Conveyor/Indexer", "Started");
            }

            // Start shooter after movement sequence
            if (shooter != null) {
                shooter.setPower(1.0);
                telemetry.addData("Shooter", "Started");
            } else {
                telemetry.addData("Shooter", "Not configured (name: 'shooter')");
            }

            // Keep indexer rotating until ball is in front of trigger
            if (indexer != null) {
                boolean ballAtTrigger = false;
                while (opModeIsActive() && !ballAtTrigger) {
                    if (indexerSensor != null && indexerSensor.isPressed()) {
                        ballAtTrigger = true;
                        indexer.setPower(0.0);
                        telemetry.addData("Indexer", "Ball at trigger - stopped");
                        telemetry.update();
                    } else {
                        telemetry.addData("Indexer", "Rotating, waiting for ball at trigger...");
                        telemetry.update();
                        sleep(100);
                    }
                }
            }

            // Push the ball by moving trigger down
            if (trigger != null) {
                trigger.setPosition(1.0); // Move trigger down to push ball
                telemetry.addData("Trigger", "Pushed down - ball firing");
                telemetry.update();
                sleep(500); // Hold trigger down for 500ms to ensure ball fires
                trigger.setPosition(0.0); // Move trigger back up
                telemetry.addData("Trigger", "Reset to up position");
                telemetry.update();
            } else {
                telemetry.addData("Trigger", "Not configured (name: 'trigger')");
                telemetry.update();
            }

            // Wait for ball to leave shooter, then turn off all motors
            sleep(1000); // Give ball time to leave shooter
            
            // Stop all motors
            if (intake != null) intake.setPower(0.0);
            if (conveyor != null) conveyor.setPower(0.0);
            if (indexer != null) indexer.setPower(0.0);
            if (shooter != null) shooter.setPower(0.0);
            
            telemetry.addData("Status", "Ball fired - all motors stopped");
            telemetry.addData("Robot", "Shutting down");
            telemetry.update();
            sleep(500);

            telemetry.addData("Status", "Complete");
            telemetry.update();
        }
    }
}
