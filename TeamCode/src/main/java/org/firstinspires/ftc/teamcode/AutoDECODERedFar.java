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
@Autonomous(name="Auto Red Far", group="Linear OpMode")
public class AutoDECODERedFar extends LinearOpMode {
    
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

    // Alliance and position configuration
    private static final String ALLIANCE = "RED";
    private static final String POSITION = "BACK";
    private static final Pose2d START_POSE = new Pose2d(12.0, -108.0, 0.0); // Starting pose for Road Runner (back position) - mirrored Y
    
    // Motor power settings
    public static final double INTAKE_POWER = 0.8;
    public static final double CONVEYOR_POWER = 1.0;
    public static final double AUTO_INDEXOR_POWER = 0.3;      // Power for automatic indexor movement
    public static final double SHOOTER_POWER = 1.0;
    public static final double SHOOTER_SERVO_POWER = 1.0;     // Positive for forward direction
    
    // Indexor position settings
    public static final double INDEXOR_TICKS = 537.7/3;              // goBILDA 312 RPM motor: 120 degrees = 179 ticks
    
    // Shooter velocity control (ticks per second) - Red alliance optimized
    public static double SHOOTER_TARGET_VELOCITY = 1550;      // Range: 1200-1800 ticks/sec (Red back position)
    public static final double SHOOTER_SPEED_THRESHOLD = 0.95; // 95% of target speed
    public static final double SHOOTER_TICKS_PER_REVOLUTION = 1020.0; // goBILDA 435 RPM motor
    
    // Speed stabilization settings
    public static final double SHOOTER_SPEED_TOLERANCE = 25;    // ticks/sec tolerance for "stable" speed
    public static final double SHOOTER_STABILIZATION_TIME = 0.5; // Seconds to wait for speed stabilization between shots
    public static final double SHOOTER_VELOCITY_CORRECTION_FACTOR = 1.02; // Slight overcorrection for consistency
    
    // Speed light control settings (using servo positions for LED control)
    public static final double LIGHT_OFF_POSITION = 0.0;      // Servo position for light off
    public static final double LIGHT_GREEN_POSITION = 0.5;    // Servo position for green light
    public static final double LIGHT_WHITE_POSITION = 1.0;    // Servo position for white light
    public static final double LIGHT_RED_POSITION = 0.75;     // Servo position for red light (alliance indicator)
    
    // Trigger servo positions
    public static final double TRIGGER_FIRE = 0.0;     // Fire position (27.0 degrees)
    public static final double TRIGGER_HOME = 0.5;     // Home position (104.4 degrees)
    
    // Autonomous timing settings
    public static final double TRIGGER_FIRE_DURATION = 0.5;   // Seconds to stay in fire position
    public static final double WAIT_BETWEEN_SHOTS = 0.3;      // Seconds to wait between shots (stabilization)
    public static final double INDEXOR_MOVE_TIMEOUT = 3.0;    // Maximum time to wait for indexor movement
    public static final double SHOOTER_SPINUP_TIMEOUT = 5.0;  // Maximum time to wait for shooter to reach speed
    public double IndexerPreviousPosition = 0.0;  // Maximum time to wait for shooter to reach speed

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
        telemetry.addData("Alliance", "🔴 RED");
        telemetry.addData("Position", "FAR");
        telemetry.addData("Start Pose", "X: %.1f\", Y: %.1f\", H: %.1f°", START_POSE.position.x, START_POSE.position.y, Math.toDegrees(START_POSE.heading.toDouble()));
        telemetry.addData("=== AUTONOMOUS SEQUENCE ===", "");
        telemetry.addData("1.", "Trajectory 1 - Move to shooting position");
        telemetry.addData("2.", "Shooter - Fire 3 shots");
        telemetry.addData("3.", "Trajectory 2 - 27\" fwd + 115° turn + 30\" fwd");
        telemetry.addData("", "");
        telemetry.addData("Shooter Speed", "%.0f ticks/sec", SHOOTER_TARGET_VELOCITY);
        telemetry.addData("Alliance Light", "🔴 Red indicator");
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
                .waitSeconds(0.1)  // Placeholder - replace with actual movement if needed
                .build();
        
        // Trajectory 2: Move 27 inches forward while turning to 115 degrees, then move 30 inches forward with intake
        // Mirrored from blue: Y coordinates negated, angles adjusted for red alliance
        trajectory2 = drive.actionBuilder(START_POSE)
                .lineToXSplineHeading(START_POSE.position.x + 27.0, Math.toRadians(115))  // Mirror: -115 becomes +115
                .afterTime(0, (telemetryPacket) -> {
                    // Start intake system during forward movement
                    intake.setPower(INTAKE_POWER);
                    conveyor.setPower(CONVEYOR_POWER);
                    indexor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    indexor.setPower(AUTO_INDEXOR_POWER);
                    return false;
                })
                .setTangent(Math.toRadians(115))  // Mirror: -115 becomes +115
                .lineToY(START_POSE.position.y - 30.0)  // Mirror: +30 becomes -30
                .stopAndAdd((telemetryPacket) -> {
                    // Stop intake system after forward movement
                    intake.setPower(0);
                    conveyor.setPower(0);
                    indexor.setPower(0);
                    return false;
                })
                .lineToYSplineHeading(START_POSE.position.y - 1, Math.toRadians(0))  // Mirror: +1 becomes -1
                .setTangent(Math.toRadians(0))
                .lineToX(START_POSE.position.x + 2.0)
                .build();

        trajectoryCloseOut = drive.actionBuilder(new Pose2d(START_POSE.position.x + 2.0, START_POSE.position.y - 1, 0))  // Mirror: +1 becomes -1
                .lineToX(30)
                .build();

        telemetry.addData("✅ Trajectory 1", "Built (ready position)");
        telemetry.addData("✅ Trajectory 2", "Built (27\" fwd + 115° turn + 30\" fwd)");
        telemetry.update();
    }
    
    private void executeAutonomousSequence() {
        telemetry.addData("🤖 AUTONOMOUS", "Starting Red Back Road Runner sequence...");
        telemetry.update();
        
        // Step 1: Execute Trajectory 1 (move to shooting position)
        telemetry.addData("🚀 STEP 1", "Executing Trajectory 1...");
        telemetry.update();
        Actions.runBlocking(trajectory1);
        
        telemetry.addData("✅ Trajectory 1", "Complete");
        telemetry.update();
        
        // Step 2: Shooter sequence - Fire 3 shots
        telemetry.addData("🚀 STEP 2", "Executing Shooter Sequence...");
        telemetry.update();
        
        startShooterSystem();
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
        
        // Step 3: Execute Trajectory 2 (move forward with turn, then forward with intake)
        telemetry.addData("🚀 STEP 3", "Executing Trajectory 2...");
        telemetry.update();
        
        // Execute trajectory 2
        Actions.runBlocking(trajectory2);
        
        telemetry.addData("✅ Trajectory 2", "Complete");
        telemetry.update();

        Actions.runBlocking(trajectoryCloseOut);
        
        telemetry.addData("✅ AUTONOMOUS", "Red Back Road Runner sequence completed!");
        telemetry.addData("🔴 Alliance", "RED");
        telemetry.addData("📍 Final Position", "27\" fwd + 115° turn + 30\" fwd");
        telemetry.addData("🎯 Shots Fired", "3 shots");
        telemetry.addData("⏱️ Status", "Autonomous finished");
        telemetry.update();
    }
    
    private void startShooterSystem() {
        telemetry.addData("🚀 STEP 1", "Starting shooter system...");
        telemetry.update();
        
        // Start shooter with optimized velocity control for consistency
        double initialVelocity = SHOOTER_TARGET_VELOCITY * SHOOTER_VELOCITY_CORRECTION_FACTOR;
        shooter.setVelocity(initialVelocity);
        
        // Start shooter servo
        shooterServo.setPower(SHOOTER_SERVO_POWER);
        
        // Start conveyor to help feed balls
        conveyor.setPower(CONVEYOR_POWER);
        
        // Set alliance indicator light
        speedLight.setPosition(LIGHT_RED_POSITION);
        
        telemetry.addData("✅ Shooter", "Started at %.0f ticks/sec (corrected)", initialVelocity);
        telemetry.addData("🎯 Target", "%.0f ticks/sec", SHOOTER_TARGET_VELOCITY);
        telemetry.addData("✅ Shooter Servo", "Running at %.1f power", SHOOTER_SERVO_POWER);
        telemetry.addData("✅ Conveyor", "Running at %.1f power", CONVEYOR_POWER);
        telemetry.addData("🔴 Alliance Light", "Red indicator active");
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
        telemetry.addData("🔥 STEP 1." + shotNumber, "Firing shot %d of 3...", shotNumber);
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
            // Continuously reapply shooter velocity to maintain consistent speed
            shooter.setVelocity(SHOOTER_TARGET_VELOCITY);
            
            double currentVelocity = shooter.getVelocity();
            double speedPercentage = currentVelocity / SHOOTER_TARGET_VELOCITY;
            double velocityError = Math.abs(currentVelocity - SHOOTER_TARGET_VELOCITY);
            
            telemetry.addData("🎯 Shot", "%d of 3", shotNumber);
            telemetry.addData("💥 Trigger", "FIRE position");
            telemetry.addData("⚡ Shooter", "%.0f ticks/sec (%.0f%%)", currentVelocity, speedPercentage * 100);
            telemetry.addData("📊 Velocity Error", "%.0f ticks/sec", velocityError);
            telemetry.addData("⏱️ Fire Time", "%.1f / %.1f seconds", fireTimer.seconds(), TRIGGER_FIRE_DURATION);
            
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
        int targetPosition = currentPosition + (int)INDEXOR_TICKS;
        
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
            speedLight.setPosition(LIGHT_RED_POSITION); // Alliance indicator while spinning up
        }
    }
    
    private String getSpeedLightStatus(double currentVelocity) {
        double speedPercentage = currentVelocity / SHOOTER_TARGET_VELOCITY;
        
        if (speedPercentage >= SHOOTER_SPEED_THRESHOLD) {
            return "🟢 GREEN (Ready)";
        } else if (speedPercentage >= 0.8) {
            return "⚪ WHITE (Close)";
        } else {
            return "🔴 RED (Alliance)";
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
