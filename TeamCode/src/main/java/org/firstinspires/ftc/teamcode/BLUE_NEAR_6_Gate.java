package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name="BLUE NEAR 6 GATE", group="Linear OpMode")
public class BLUE_NEAR_6_GATE extends LinearOpMode {
    
    // Declare motors
    private DcMotorEx indexor;
    private DcMotorEx intake;
    private DcMotorEx conveyor;
    private DcMotorEx shooter;
    
    // Declare servos
    private CRServo shooterServo;
    private Servo speedLight;
    private Servo triggerServo;
    
    // Declare mecanum drive
    private MecanumDrive drive;
    
    // Pre-built actions
    private Action trajectory1;
    private Action trajectory2;
    private Action trajectoryCloseOut;

    public boolean ranTrajectory2 = false;

    // Alliance and position configuration
    private static final String ALLIANCE = "BLUE";
    private static final String POSITION = "BACK";
    private static final Pose2d START_POSE = new Pose2d(12.0, 108.0, 0.0); // Starting pose for Road Runner (back position)
    
    // Motor power settings
    public static final double INTAKE_POWER = 0.8;
    public static final double CONVEYOR_POWER = 1.0;
    public static final double AUTO_INDEXOR_POWER = 0.6;      // Increased from 0.3 for faster movement
    public static double INDEXOR_INTAKE_VELOCITY = 500.0;     // Indexor velocity during intake (ticks/sec) - tunable for optimal intake
    public static final double SHOOTER_POWER = 1.0;
    public static final double SHOOTER_SERVO_POWER = 1.0;     // Positive for forward direction
    

    // Shooter PID coefficients for velocity control (from TeleOpDECODE)
    public static double VELOCITY_P = 6.0;    // Proportional coefficient (increased from 4 for faster response)
    public static double VELOCITY_I = 0.15;   // Integral coefficient
    public static double VELOCITY_D = 0.5;    // Derivative coefficient (increased from 0.3 for better damping)
    public static double VELOCITY_F = 13.0;   // Feedforward coefficient
    
    // Voltage boost settings
    public static final double BOOST_VOLTAGE_MULTIPLIER = 1.35;    // 35% voltage boost for startup
    public static final double BOOST_DURATION = 0.4;                // 400 milliseconds boost duration
    
    
    
    // Indexor position settings
    public static final double INDEXOR_TICKS = 537.7/3;              // goBILDA 312 RPM motor: 120 degrees = 179 ticks
    
    // Shooter velocity control (ticks per second) - Blue alliance optimized
    public static double SHOOTER_TARGET_VELOCITY = 1280;      // Range: 1200-1800 ticks/sec (Blue back position)
    public static final double SHOOTER_SPEED_THRESHOLD = 0.95; // 95% of target speed
    public static final double SHOOTER_TICKS_PER_REVOLUTION = 1020.0; // goBILDA 435 RPM motor
    
    // Speed stabilization settings
    public static final double SHOOTER_SPEED_TOLERANCE = 25;    // ticks/sec tolerance for "stable" speed
    public static final double SHOOTER_STABILIZATION_TIME = 0.2; // Seconds to wait for speed stabilization (reduced from 0.5s)
    public static final double SHOOTER_VELOCITY_CORRECTION_FACTOR = 1.02; // Slight overcorrection for consistency
    
    // Speed light control settings (using servo positions for LED control)
    public static final double LIGHT_OFF_POSITION = 0.0;      // Servo position for light off
    public static final double LIGHT_GREEN_POSITION = 0.5;    // Servo position for green light
    public static final double LIGHT_WHITE_POSITION = 1.0;    // Servo position for white light
    public static final double LIGHT_BLUE_POSITION = 0.25;    // Servo position for blue light (alliance indicator)
    
    // Trigger servo positions
    public static final double TRIGGER_FIRE = 0.0;     // Fire position (27.0 degrees)
    public static final double TRIGGER_HOME = 0.5;     // Home position (104.4 degrees)
    
    // Autonomous timing settings
    public static final double TRIGGER_FIRE_DURATION = 0.3;   // Seconds to stay in fire position (reduced from 0.5s)
    public static final double WAIT_BETWEEN_SHOTS = 0.15;     // Seconds to wait between shots (reduced from 0.3s)
    public static final double INDEXOR_MOVE_TIMEOUT = 3.0;    // Maximum time to wait for indexor movement
    public static final double SHOOTER_SPINUP_TIMEOUT = 5.0;  // Maximum time to wait for shooter to reach speed
    public double IndexerPreviousPosition = 0.0;  // Maximum time to wait for shooter to reach speed

 // Road Runner trajectory settings
    public static final double REARWARD_DISTANCE = 51.0;      // Distance to move left (inches)
    public static final double LEFTWARD_DISTANCE = 24.0;      // Distance to move forward (inches)
   
    // Road Runner trajectory settings
    public static final double FORWARD_DISTANCE = 40.0;       // Distance to move sideways (inches)
    
    @Override
    public void runOpMode() {
        // Initialize motors and Road Runner drive
        initializeMotors();

        // Build all trajectory actions during initialization
        buildTrajectoryActions();
        
        // Display autonomous sequence
        telemetry.addData("Status", "Auto Far - Road Runner Initialized");
        telemetry.addData("Alliance", "🔵 BLUE");
        telemetry.addData("Position", "FAR");
        telemetry.addData("Start Pose", "X: %.1f\", Y: %.1f\", H: %.1f°", START_POSE.position.x, START_POSE.position.y, Math.toDegrees(START_POSE.heading.toDouble()));
        telemetry.addData("=== AUTONOMOUS SEQUENCE ===", "");
        telemetry.addData("1.", "Trajectory 1 - Move to shooting position");
        telemetry.addData("2.", "Shooter - Fire 3 shots");
        telemetry.addData("3.", "Trajectory 2 - 30\" fwd + 130° turn + 10\" rear");
        telemetry.addData("", "");
        telemetry.addData("Shooter Speed", "%.0f ticks/sec", SHOOTER_TARGET_VELOCITY);
        telemetry.addData("Alliance Light", "🔵 Blue indicator");
        telemetry.addData("Total Time", "~8-12 seconds");
        telemetry.addData("", "");
        telemetry.addData("Drive System", "Road Runner with GoBilda Pinpoint");
        telemetry.addData("✅ Actions Built", "Ready to execute");
        telemetry.update();
        
        waitForStart();
        
        if (opModeIsActive()) {
            executeAutonomousSequence();
        }
    }
    
    private void buildTrajectoryActions() {
        telemetry.addData("🏗️ Building Actions", "Creating trajectory paths...");
        telemetry.update();
        
        // Trajectory 1: Initial movement (if needed - currently staying at start)
        // You can modify this to move to a specific shooting position
        trajectory1 = drive.actionBuilder(START_POSE)
                .lineToX(START_POSE.position.x - REARWARD_DISTANCE)  // Move left 50 inches (same as Blue)
                .build();

        
        // Trajectory 2: Move 30 inches forward while turning to 130 degrees, then move 10 inches rearward with intake
        trajectory2 = drive.actionBuilder(new Pose2d(START_POSE.position.x - REARWARD_DISTANCE, START_POSE.position.y, START_POSE.heading.toDouble()))
                .turnTo(Math.toRadians(-134))
                .afterTime(0, (telemetryPacket) -> {
                    // Start indexor only - intake and conveyor already running
                    indexor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    indexor.setVelocity(INDEXOR_INTAKE_VELOCITY);
                    return false;
                })
                
                .strafeToLinearHeading(new Vector2d(START_POSE.position.x - REARWARD_DISTANCE - 4.0, START_POSE.position.y+2), Math.toRadians(-134)) //10.0
                .setTangent(Math.toRadians(-134))
                .lineToY(START_POSE.position.y + 31.0) //29.0

               .stopAndAdd((telemetryPacket) -> {
                    // Stop indexor only - keep intake and conveyor running
                    indexor.setVelocity(0);
                    return false;
                })
                .lineToYSplineHeading(START_POSE.position.y+3, Math.toRadians(0))
                .build();

         trajectoryCloseOut = drive.actionBuilder(new Pose2d(START_POSE.position.x - REARWARD_DISTANCE, START_POSE.position.y, START_POSE.heading.toDouble()))
                .strafeToLinearHeading(new Vector2d(START_POSE.position.x - REARWARD_DISTANCE, START_POSE.position.y + LEFTWARD_DISTANCE), START_POSE.heading.toDouble())  // Strafe backward 24 inches (reversed direction)
                .build();


        telemetry.addData("✅ Trajectory 1", "Built (ready position)");
        telemetry.addData("✅ Trajectory 2", "Built (30\" fwd + 130° turn + 10\" rear)");
        telemetry.update();
    }
    
    private void executeAutonomousSequence() {
        telemetry.addData("🤖 AUTONOMOUS", "Starting Blue Back Road Runner sequence...");
        telemetry.update();
        
        // Start shooter immediately - it will spin up during trajectory1
        startShooterSystem();
        
        // Start intake and conveyor - keep them running throughout
        intake.setPower(AUTO_INDEXOR_POWER);
        conveyor.setPower(CONVEYOR_POWER);
        
        // Step 1: Execute Trajectory 1 (move to shooting position)
        telemetry.addData("🚀 STEP 1", "Executing Trajectory 1...");
        telemetry.update();
        Actions.runBlocking(trajectory1);
        
        telemetry.addData("✅ Trajectory 1", "Complete");
        telemetry.update();
        
        // Step 2: Shooter sequence - Fire 3 shots
        telemetry.addData("🚀 STEP 2", "Executing Shooter Sequence...");
        telemetry.update();
        
        // Shooter should be ready or nearly ready by now
        waitForShooterSpeed();
        
        fireShot(1);
        moveIndexorToNextPosition();
        
        fireShot(2);
        moveIndexorToNextPosition();
        
        fireShot(3);
        moveIndexorToNextPosition();
        
        //stopShooterSystem();
        
        telemetry.addData("✅ Shooter Sequence", "Complete - 3 shots fired");
        telemetry.update();
        
        // Step 3: Execute Trajectory 2 (move forward with turn, then rearward with intake)
        telemetry.addData("🚀 STEP 3", "Executing Trajectory 2...");
        telemetry.update();
        
        // Execute first part of trajectory 2 (forward movement with turn)
        Actions.runBlocking(trajectory2);
        ranTrajectory2 = true;

        // DIAGNOSTIC PAUSE: Display indexer position information after picking balls
        double currentPosition = indexor.getCurrentPosition();
        double indexerCorrection = 0.0;
        if (currentPosition > IndexerPreviousPosition + 5) {
            indexerCorrection = INDEXOR_TICKS - (currentPosition % INDEXOR_TICKS);
        }
        double nextTargetPosition = IndexerPreviousPosition + INDEXOR_TICKS;

        //sleep(5000); // 5 second pause to review

        moveIndexorToNextPosition();

        fireShot(4);

        moveIndexorToNextPosition();

        fireShot(5);

        moveIndexorToNextPosition();
        
        // Turn 90 degrees and back after shot 5
        telemetry.addData("🔄 TURNING", "Rotating 90 degrees...");
        telemetry.update();
        
        Action turnSequence = drive.actionBuilder(new Pose2d(START_POSE.position.x - REARWARD_DISTANCE, START_POSE.position.y + 3, Math.toRadians(0)))
                .turnTo(Math.toRadians(90))
                .turnTo(Math.toRadians(0))
                .build();
        
        Actions.runBlocking(turnSequence);
        
        telemetry.addData("✅ Turn Complete", "Returning to shooting...");
        telemetry.update();

        fireShot(6);

        moveIndexorToNextPosition();
        
        telemetry.addData("✅ Shooting Complete", "All 6 shots fired, starting movement");
        telemetry.update();

        Actions.runBlocking(trajectoryCloseOut);


        telemetry.addData("✅ Trajectory 2", "Complete");
        telemetry.update();
        
        telemetry.addData("✅ AUTONOMOUS", "Blue Back Road Runner sequence completed!");
        telemetry.addData("🔵 Alliance", "BLUE");
        telemetry.addData("📍 Final Position", "30\" fwd + 130° turn + 10\" rear");
        telemetry.addData("🎯 Shots Fired", "3 shots");
        telemetry.addData("⏱️ Status", "Autonomous finished");
        telemetry.update();
    }
    
    private void startShooterSystem() {
        telemetry.addData("🚀 STEP 1", "Starting shooter system...");
        telemetry.update();
        
        // Start shooter with voltage boost for faster spin-up
        double boostedVelocity = SHOOTER_TARGET_VELOCITY * BOOST_VOLTAGE_MULTIPLIER;
        shooter.setVelocity(boostedVelocity);
        
        // Start shooter servo
        shooterServo.setPower(SHOOTER_SERVO_POWER);
        
        // Start conveyor to help feed balls
        conveyor.setPower(CONVEYOR_POWER);
        
        // Set alliance indicator light
        speedLight.setPosition(LIGHT_BLUE_POSITION);
        
        telemetry.addData("✅ Shooter", "Started with BOOST: %.0f ticks/sec", boostedVelocity);
        telemetry.addData("🎯 Target", "%.0f ticks/sec", SHOOTER_TARGET_VELOCITY);
        telemetry.update();
        
        // Wait for boost duration, then return to normal velocity
        ElapsedTime boostTimer = new ElapsedTime();
        boostTimer.reset();
        
        while (opModeIsActive() && boostTimer.seconds() < BOOST_DURATION) {
            sleep(50);
        }
        
        // Return to normal velocity after boost
        shooter.setVelocity(SHOOTER_TARGET_VELOCITY);
        telemetry.addData("✅ Boost Complete", "Normal velocity: %.0f ticks/sec", SHOOTER_TARGET_VELOCITY);
        telemetry.update();
    }
    
    private void waitForShooterSpeed() {
        telemetry.addData("⏳ STEP", "Waiting for shooter to reach speed...");
        telemetry.update();
        
        ElapsedTime timeout = new ElapsedTime();
        timeout.reset();
        
        while (opModeIsActive() && timeout.seconds() < SHOOTER_SPINUP_TIMEOUT) {
            double currentVelocity = shooter.getVelocity();
            double speedPercentage = currentVelocity / SHOOTER_TARGET_VELOCITY;
            
            // Update speed light
            updateSpeedLight(currentVelocity);
            
            telemetry.addData("🎯 Target Speed", "%.0f ticks/sec", SHOOTER_TARGET_VELOCITY);
            telemetry.addData("⚡ Current Speed", "%.0f ticks/sec (%.0f%%)", 
                currentVelocity, speedPercentage * 100);
            telemetry.addData("💡 Speed Light", getSpeedLightStatus(currentVelocity));
            telemetry.addData("⏱️ Elapsed", "%.1f / %.1f seconds", timeout.seconds(), SHOOTER_SPINUP_TIMEOUT);
            
            // Check if we've reached target speed
            if (speedPercentage >= SHOOTER_SPEED_THRESHOLD) {
                telemetry.addData("✅ READY", "Shooter at target speed!");
                telemetry.update();
                
                // Wait for speed stabilization
                stabilizeShooterSpeed();
                return;
            }
            
            telemetry.update();
            sleep(50);
        }
        
        // Timeout reached
        telemetry.addData("⚠️ TIMEOUT", "Proceeding with current speed");
        telemetry.update();
    }
    
    private void stabilizeShooterSpeed() {
        telemetry.addData("⚖️ STABILIZING", "Ensuring shooter speed consistency...");
        telemetry.update();
        
        ElapsedTime stabilizationTimer = new ElapsedTime();
        stabilizationTimer.reset();
        
        double lastVelocity = shooter.getVelocity();
        boolean speedStable = false;
        
        while (opModeIsActive() && stabilizationTimer.seconds() < SHOOTER_STABILIZATION_TIME) {
            double currentVelocity = shooter.getVelocity();
            double velocityDifference = Math.abs(currentVelocity - lastVelocity);
            double targetDifference = Math.abs(currentVelocity - SHOOTER_TARGET_VELOCITY);
            
            // Check if speed is stable (small variations)
            speedStable = (velocityDifference < SHOOTER_SPEED_TOLERANCE) && 
                         (targetDifference < SHOOTER_SPEED_TOLERANCE);
            
            // Apply velocity correction if needed
            if (targetDifference > SHOOTER_SPEED_TOLERANCE) {
                double correctedVelocity = SHOOTER_TARGET_VELOCITY * SHOOTER_VELOCITY_CORRECTION_FACTOR;
                shooter.setVelocity(correctedVelocity);
                
                telemetry.addData("🔧 CORRECTING", "Adjusting to %.0f ticks/sec", correctedVelocity);
            }
            
            telemetry.addData("🎯 Target", "%.0f ticks/sec", SHOOTER_TARGET_VELOCITY);
            telemetry.addData("⚡ Current", "%.0f ticks/sec", currentVelocity);
            telemetry.addData("📊 Variation", "%.0f ticks/sec", velocityDifference);
            telemetry.addData("🎯 Target Diff", "%.0f ticks/sec", targetDifference);
            telemetry.addData("⚖️ Stable", speedStable ? "✅ YES" : "⏳ Stabilizing...");
            telemetry.addData("⏱️ Stabilizing", "%.1f / %.1f seconds", 
                stabilizationTimer.seconds(), SHOOTER_STABILIZATION_TIME);
            telemetry.update();
            
            // If speed is stable for a reasonable time, we can exit early
            if (speedStable && stabilizationTimer.seconds() > 0.3) {
                telemetry.addData("✅ STABILIZED", "Shooter speed consistent!");
                telemetry.update();
                return;
            }
            
            lastVelocity = currentVelocity;
            sleep(100); // Check every 100ms for stability
        }
        
        telemetry.addData("✅ STABILIZATION", "Complete - Ready to fire!");
        telemetry.update();
    }
    
    private void fireShot(int shotNumber) {
        telemetry.addData("🔥 STEP 1." + shotNumber, "Firing shot %d of 3...", shotNumber);
        telemetry.update();
        
        // Pre-fire velocity check and correction
        double preFire = shooter.getVelocity();
        double targetDifference = Math.abs(preFire - SHOOTER_TARGET_VELOCITY);
        
        if (targetDifference > SHOOTER_SPEED_TOLERANCE) {
            telemetry.addData("🔧 PRE-FIRE", "Correcting velocity: %.0f → %.0f", preFire, SHOOTER_TARGET_VELOCITY);
            shooter.setVelocity(SHOOTER_TARGET_VELOCITY * SHOOTER_VELOCITY_CORRECTION_FACTOR);
            telemetry.update();
            sleep(100); // Brief stabilization (reduced from 200ms)
        }
        
        // Move trigger to fire position
        triggerServo.setPosition(TRIGGER_FIRE);
        
        ElapsedTime fireTimer = new ElapsedTime();
        fireTimer.reset();
        
        // Wait for fire duration with velocity monitoring (optimized)
        while (opModeIsActive() && fireTimer.seconds() < TRIGGER_FIRE_DURATION) {
            // Continuously reapply shooter velocity to maintain consistent speed
            shooter.setVelocity(SHOOTER_TARGET_VELOCITY);
            sleep(10);  // Reduced from 50ms for faster response
        }
        //Trigger intermittent firing
        triggerServo.setPosition(0.25);
        triggerServo.setPosition(TRIGGER_FIRE);
        sleep(50);
        // Return trigger to home position
        triggerServo.setPosition(TRIGGER_HOME);
        
        // Post-fire velocity check
        double postFire = shooter.getVelocity();
        telemetry.addData("✅ Shot %d", "Fired successfully!", shotNumber);
        telemetry.addData("📊 Post-Fire Speed", "%.0f ticks/sec", postFire);
        telemetry.update();
        
        // Wait between shots
        sleep((long)(WAIT_BETWEEN_SHOTS * 1000));
    }
    
    private void moveIndexorToNextPosition() {
        telemetry.addData("🔄 INDEXOR", "Moving to next position...");
        telemetry.update();
        
        // Get current position and calculate target
        double currentPosition = indexor.getCurrentPosition();
        double targetPosition;
        
        if (ranTrajectory2) {
            double correction = currentPosition % INDEXOR_TICKS;
            targetPosition = currentPosition + INDEXOR_TICKS - correction;

            telemetry.addData("🎯 Target Position", "%.1f ticks", targetPosition);
            telemetry.addData("📍 Current Position", "%.1f ticks", (double)currentPosition);
            telemetry.addData("📍 Correction", "%.1f ticks", correction);
            telemetry.addData("📊 Calculation", "%.1f + %.1f - %.1f = %.1f", 
                (double)currentPosition, INDEXOR_TICKS, correction, targetPosition);
            telemetry.update();

            ranTrajectory2 = false;
        } else {
            targetPosition = IndexerPreviousPosition + INDEXOR_TICKS;
        }

        IndexerPreviousPosition = targetPosition;

            // Set indexor to run to position
        indexor.setTargetPosition((int)targetPosition);
        indexor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        indexor.setPower(AUTO_INDEXOR_POWER);
        
        ElapsedTime indexorTimer = new ElapsedTime();
        indexorTimer.reset();
        
        // Wait for indexor to reach position (optimized for speed)
        while (opModeIsActive() && indexor.isBusy() && indexorTimer.seconds() < INDEXOR_MOVE_TIMEOUT) {
            sleep(10);  // Reduced from 50ms to 10ms for faster response
        }

        // Stop indexor - conveyor keeps running
        indexor.setPower(0);
        indexor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("✅ Indexor", "Position advanced");
        telemetry.update();
    }
    
    private void stopShooterSystem() {
        telemetry.addData("🛑 STEP", "Stopping shooter system...");
        telemetry.update();
        
        // Stop shooter
        shooter.setPower(0);
        
        // Stop shooter servo
        shooterServo.setPower(0);
        
        // Stop conveyor
        conveyor.setPower(0);
        
        // Turn off speed light
        speedLight.setPosition(LIGHT_OFF_POSITION);
        
        // Return trigger to home position
        triggerServo.setPosition(TRIGGER_HOME);
        
        telemetry.addData("✅ Shooter", "Stopped");
        telemetry.addData("✅ Shooter Servo", "Stopped");
        telemetry.addData("✅ Conveyor", "Stopped");
        telemetry.addData("✅ Speed Light", "Off");
        telemetry.addData("✅ Trigger", "Home position");
        telemetry.update();
    }
    
    private void updateSpeedLight(double currentVelocity) {
        double speedPercentage = currentVelocity / SHOOTER_TARGET_VELOCITY;
        
        if (speedPercentage >= SHOOTER_SPEED_THRESHOLD) {
            speedLight.setPosition(LIGHT_GREEN_POSITION); // Ready
        } else if (speedPercentage >= 0.8) {
            speedLight.setPosition(LIGHT_WHITE_POSITION); // Getting close
        } else {
            speedLight.setPosition(LIGHT_BLUE_POSITION); // Alliance indicator while spinning up
        }
    }
    
    private String getSpeedLightStatus(double currentVelocity) {
        double speedPercentage = currentVelocity / SHOOTER_TARGET_VELOCITY;
        
        if (speedPercentage >= SHOOTER_SPEED_THRESHOLD) {
            return "🟢 GREEN (Ready)";
        } else if (speedPercentage >= 0.8) {
            return "⚪ WHITE (Close)";
        } else {
            return "🔵 BLUE (Alliance)";
        }
    }
    
    private void initializeMotors() {
        // Initialize all motors
        indexor = hardwareMap.get(DcMotorEx.class, "indexor");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        conveyor = hardwareMap.get(DcMotorEx.class, "conveyor");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        
        // Set custom PID coefficients for shooter velocity control
        shooter.setVelocityPIDFCoefficients(VELOCITY_P, VELOCITY_I, VELOCITY_D, VELOCITY_F);
        
        // Initialize servos
        shooterServo = hardwareMap.get(CRServo.class, "shooterServo");
        speedLight = hardwareMap.get(Servo.class, "speedLight");
        triggerServo = hardwareMap.get(Servo.class, "triggerServo");
        
        // Initialize Road Runner drive
        drive = new MecanumDrive(hardwareMap, START_POSE);
        
        // Set motor directions
        indexor.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);
        conveyor.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.REVERSE);
        
        // Set servo directions
        shooterServo.setDirection(DcMotorSimple.Direction.REVERSE);
        speedLight.setDirection(Servo.Direction.FORWARD);
        triggerServo.setDirection(Servo.Direction.FORWARD);
        
        // Initialize speed light to off position
        speedLight.setPosition(LIGHT_OFF_POSITION);
        
        // Initialize trigger servo to home (safe) position
        triggerServo.setPosition(TRIGGER_HOME);
        
        // Set zero power behavior
        indexor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        conveyor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        // Reset encoders
        indexor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        conveyor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Set indexor to use encoder
        indexor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}