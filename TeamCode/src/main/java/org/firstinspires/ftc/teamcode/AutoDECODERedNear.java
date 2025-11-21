package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
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
@Autonomous(name = "Auto Red Near", group = "Red Alliance")
public class AutoRedNear extends LinearOpMode {
    
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
    
    // Alliance and position configuration
    private static final String ALLIANCE = "RED";
    private static final String POSITION = "FRONT";
    private static final Pose2d START_POSE = new Pose2d(12.0, -132.0, 0.0); // Starting pose for Road Runner - Red alliance
    
    // Motor power settings
    public static final double INTAKE_POWER = 0.8;
    public static final double CONVEYOR_POWER = 1.0;
    public static final double AUTO_INDEXOR_POWER = 0.1;      // Power for automatic indexor movement
    public static final double SHOOTER_POWER = 1.0;
    public static final double SHOOTER_SERVO_POWER = 1.0;     // Positive for forward direction
    
    // Indexor position settings
    public static final int INDEXOR_TICKS = 179;              // goBILDA 312 RPM motor: 120 degrees = 179 ticks
    
    // Shooter velocity control (ticks per second) - Blue alliance optimized
    public static double SHOOTER_TARGET_VELOCITY = 1150;      // Range: 1200-1800 ticks/sec (Blue near position)
    public static final double SHOOTER_SPEED_THRESHOLD = 0.95; // 95% of target speed
    public static final double SHOOTER_TICKS_PER_REVOLUTION = 1020.0; // goBILDA 435 RPM motor
    
    // Speed stabilization settings
    public static final double SHOOTER_SPEED_TOLERANCE = 25;    // ticks/sec tolerance for "stable" speed
    public static final double SHOOTER_STABILIZATION_TIME = 1.5; // Seconds to wait for speed stabilization between shots
    public static final double SHOOTER_VELOCITY_CORRECTION_FACTOR = 1.02; // Slight overcorrection for consistency
    
    // Speed light control settings (using servo positions for LED control)
    public static final double LIGHT_OFF_POSITION = 0.0;      // Servo position for light off
    public static final double LIGHT_GREEN_POSITION = 0.5;    // Servo position for green light
    public static final double LIGHT_WHITE_POSITION = 1.0;    // Servo position for white light
    public static final double LIGHT_BLUE_POSITION = 0.25;    // Servo position for blue light (alliance indicator)
    
    // Trigger servo positions
    public static final double TRIGGER_FIRE = 0.10;     // Fire position
    public static final double TRIGGER_HOME = 0.76;     // Home/safe position
    
    // Autonomous timing settings
    public static final double TRIGGER_FIRE_DURATION = 0.5;   // Seconds to stay in fire position
    public static final double WAIT_BETWEEN_SHOTS = 3.0;      // Seconds to wait between shots (stabilization)
    public static final double INDEXOR_MOVE_TIMEOUT = 3.0;    // Maximum time to wait for indexor movement
    public static final double SHOOTER_SPINUP_TIMEOUT = 5.0;  // Maximum time to wait for shooter to reach speed
    
    // Road Runner trajectory settings
    public static final double REARWARD_DISTANCE = 50.0;      // Distance to move left (inches)
    public static final double LEFTWARD_DISTANCE = 24.0;      // Distance to move forward (inches)
    
    @Override
    public void runOpMode() {
        // Initialize motors and Road Runner drive
        initializeMotors();
        
        // Display autonomous sequence
        telemetry.addData("Status", "Auto Blue Near - Road Runner Initialized");
        telemetry.addData("Alliance", "🔵 BLUE");
        telemetry.addData("Position", "NEAR");
        telemetry.addData("Start Pose", "X: %.1f\", Y: %.1f\", H: %.1f°", START_POSE.position.x, START_POSE.position.y, Math.toDegrees(START_POSE.heading.toDouble()));
        telemetry.addData("=== AUTONOMOUS SEQUENCE ===", "");
        telemetry.addData("1.", "Move left 50 inches using Road Runner");
        telemetry.addData("2.", "Start shooter + fire 3 shots");
        telemetry.addData("3.", "Move forward 12 inches using Road Runner");
        telemetry.addData("", "");
        telemetry.addData("Shooter Speed", "%.0f ticks/sec", SHOOTER_TARGET_VELOCITY);
        telemetry.addData("Alliance Light", "🔵 Blue indicator");
        telemetry.addData("Total Time", "~20-25 seconds");
        telemetry.addData("", "");
        telemetry.addData("Drive System", "Road Runner with GoBilda Pinpoint");
        telemetry.update();
        
        waitForStart();
        
        if (opModeIsActive()) {
            executeAutonomousSequence();
        }
    }
    
    private void executeAutonomousSequence() {
        telemetry.addData("🤖 AUTONOMOUS", "Starting Red Near Road Runner sequence...");
        telemetry.update();
        
        // Create trajectories - Red alliance with same first trajectory as Blue
        Action moveRearward = drive.actionBuilder(START_POSE)
                .lineToX(START_POSE.position.x - REARWARD_DISTANCE)  // Move left 50 inches (same as Blue)
                .build();
        
        Action moveLeft = drive.actionBuilder(new Pose2d(START_POSE.position.x - REARWARD_DISTANCE, START_POSE.position.y, START_POSE.heading.toDouble()))
                .strafeToLinearHeading(new Vector2d(START_POSE.position.x - REARWARD_DISTANCE, START_POSE.position.y - LEFTWARD_DISTANCE), START_POSE.heading.toDouble())  // Strafe backward 24 inches (reversed direction)
                .build();
        
        // Step 1: Move left 32 inches
        telemetry.addData("🚀 STEP 1", "Moving left %.1f inches...", REARWARD_DISTANCE);
        telemetry.update();
        Actions.runBlocking(moveRearward);
        
        telemetry.addData("✅ STEP 1", "Left movement completed");
        telemetry.update();
        sleep(500);
        
        // Step 2: Start shooter and fire 3 shots
        startShooterSystem();
        waitForShooterSpeed();
        
        fireShot(1);
        moveIndexorToNextPosition();
        waitForShooterSpeed();
        
        fireShot(2);
        moveIndexorToNextPosition();
        waitForShooterSpeed();
        
        fireShot(3);
        
        stopShooterSystem();
        
        // Step 3: Move backward 24 inches
        telemetry.addData("🚀 STEP 3", "Moving backward %.1f inches...", LEFTWARD_DISTANCE);
        telemetry.update();
        Actions.runBlocking(moveLeft);
        
        telemetry.addData("✅ AUTONOMOUS", "Red Near Road Runner sequence completed!");
        telemetry.addData("� Alliance", "RED");
        telemetry.addData("📍 Final Position", "50\" left + 24\" backward from start");
        telemetry.addData("🎯 Shots Fired", "3 shots");
        telemetry.addData("⏱️ Status", "Autonomous finished");
        telemetry.update();
    }
    
    private void startShooterSystem() {
        telemetry.addData("🚀 STEP 2", "Starting shooter system...");
        telemetry.update();
        
        // Start shooter with optimized velocity control for consistency
        double initialVelocity = SHOOTER_TARGET_VELOCITY * SHOOTER_VELOCITY_CORRECTION_FACTOR;
        shooter.setVelocity(initialVelocity);
        
        // Start shooter servo
        shooterServo.setPower(SHOOTER_SERVO_POWER);
        
        // Start conveyor to help feed balls
        conveyor.setPower(CONVEYOR_POWER);
        
        // Set alliance indicator light
        speedLight.setPosition(LIGHT_BLUE_POSITION);
        
        telemetry.addData("✅ Shooter", "Started at %.0f ticks/sec (corrected)", initialVelocity);
        telemetry.addData("🎯 Target", "%.0f ticks/sec", SHOOTER_TARGET_VELOCITY);
        telemetry.addData("✅ Shooter Servo", "Running at %.1f power", SHOOTER_SERVO_POWER);
        telemetry.addData("✅ Conveyor", "Running at %.1f power", CONVEYOR_POWER);
        telemetry.addData("🔵 Alliance Light", "Blue indicator active");
        telemetry.update();
    }
    
    private void waitForShooterSpeed() {
        telemetry.addData("⏳ STEP 4", "Waiting for shooter to reach speed...");
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
            if (speedStable && stabilizationTimer.seconds() > 0.5) {
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
        telemetry.addData("🔥 STEP 5." + shotNumber, "Firing shot %d of 3...", shotNumber);
        telemetry.update();
        
        // Pre-fire velocity check and correction
        double preFire = shooter.getVelocity();
        double targetDifference = Math.abs(preFire - SHOOTER_TARGET_VELOCITY);
        
        if (targetDifference > SHOOTER_SPEED_TOLERANCE) {
            telemetry.addData("🔧 PRE-FIRE", "Correcting velocity: %.0f → %.0f", preFire, SHOOTER_TARGET_VELOCITY);
            shooter.setVelocity(SHOOTER_TARGET_VELOCITY * SHOOTER_VELOCITY_CORRECTION_FACTOR);
            telemetry.update();
            sleep(200); // Brief stabilization
        }
        
        // Move trigger to fire position
        triggerServo.setPosition(TRIGGER_FIRE);
        
        ElapsedTime fireTimer = new ElapsedTime();
        fireTimer.reset();
        
        // Wait for fire duration with velocity monitoring
        while (opModeIsActive() && fireTimer.seconds() < TRIGGER_FIRE_DURATION) {
            double currentVelocity = shooter.getVelocity();
            double speedPercentage = currentVelocity / SHOOTER_TARGET_VELOCITY;
            double velocityError = Math.abs(currentVelocity - SHOOTER_TARGET_VELOCITY);
            
            telemetry.addData("🎯 Shot", "%d of 3", shotNumber);
            telemetry.addData("💥 Trigger", "FIRE position");
            telemetry.addData("⚡ Shooter", "%.0f ticks/sec (%.0f%%)", currentVelocity, speedPercentage * 100);
            telemetry.addData("📊 Velocity Error", "%.0f ticks/sec", velocityError);
            telemetry.addData("⏱️ Fire Time", "%.1f / %.1f seconds", fireTimer.seconds(), TRIGGER_FIRE_DURATION);
            
            // Real-time velocity correction during firing
            if (velocityError > SHOOTER_SPEED_TOLERANCE) {
                telemetry.addData("🔧 CORRECTING", "Adjusting during fire");
                shooter.setVelocity(SHOOTER_TARGET_VELOCITY * SHOOTER_VELOCITY_CORRECTION_FACTOR);
            }
            
            telemetry.update();
            sleep(50);
        }
        
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
        int currentPosition = indexor.getCurrentPosition();
        int targetPosition = currentPosition + INDEXOR_TICKS;
        
        // Set indexor to run to position
        indexor.setTargetPosition(targetPosition);
        indexor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        indexor.setPower(AUTO_INDEXOR_POWER);
        
        ElapsedTime indexorTimer = new ElapsedTime();
        indexorTimer.reset();
        
        // Wait for indexor to reach position
        while (opModeIsActive() && indexor.isBusy() && indexorTimer.seconds() < INDEXOR_MOVE_TIMEOUT) {
            telemetry.addData("🎯 Target Position", "%d ticks", targetPosition);
            telemetry.addData("📍 Current Position", "%d ticks", indexor.getCurrentPosition());
            telemetry.addData("🔄 Indexor Status", indexor.isBusy() ? "Moving..." : "Complete");
            telemetry.addData("⏱️ Elapsed", "%.1f / %.1f seconds", indexorTimer.seconds(), INDEXOR_MOVE_TIMEOUT);
            telemetry.update();
            sleep(50);
        }
        
        // Stop indexor
        indexor.setPower(0);
        indexor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        telemetry.addData("✅ Indexor", "Position advanced");
        telemetry.update();
    }
    
    private void stopShooterSystem() {
        telemetry.addData("🛑 STEP 6", "Stopping shooter system...");
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