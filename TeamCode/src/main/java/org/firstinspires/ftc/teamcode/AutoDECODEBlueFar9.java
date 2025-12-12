package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name="Auto Blue Far 9 balls", group="Linear OpMode")
public class AutoDECODEBlueFar9 extends LinearOpMode {
    
    // Declare motors
    private DcMotorEx indexor;
    private DcMotorEx intake;
    private DcMotorEx conveyor;
    private DcMotorEx shooter;
    
    // Declare servos
    private CRServo shooterServo;
    private Servo triggerServo;
    
    // Declare mecanum drive
    private MecanumDrive drive;
    
    // Pre-built actions
    private Action trajectory1;
    private Action trajectory2;
    private Action trajectoryCloseOut;
    private Action trajectoryCloseOutFinal;

    public boolean ranTrajectory2 = false;

    // Alliance and position configuration
    private static final String ALLIANCE = "BLUE";
    private static final String POSITION = "BACK";
    private static final Pose2d START_POSE = new Pose2d(12.0, 108.0, 0.0); // Starting pose for Road Runner (back position)

    // Motor power settings
    public static final double INTAKE_POWER = 0.5;
    public static final double CONVEYOR_POWER = 1.0;
    public static final double AUTO_INDEXOR_POWER = 0.3;      // Power for automatic indexor movement (increased to 0.7 for faster advancement)
    public static double INDEXOR_INTAKE_VELOCITY = 520.0;     // Velocity for indexor during intake (ticks/sec) - tunable for correct ball intake speed
    public static final double SHOOTER_POWER = 1.0;
    public static final double SHOOTER_SERVO_POWER = 1.0;     // Positive for forward direction
    
    // Shooter PID coefficients for velocity control (from TeleOpDECODE)
    public static double VELOCITY_P = 6.0;    // Proportional coefficient (increased from 4 for faster response)
    public static double VELOCITY_I = 0.15;   // Integral coefficient
    public static double VELOCITY_D = 0.5;    // Derivative coefficient (increased from 0.3 for better damping)
    public static double VELOCITY_F = 13.0;   // Feedforward coefficient
    
    // Voltage boost settings
    public static final double BOOST_VOLTAGE_MULTIPLIER = 1.35;    // 25% voltage boost for startup
    public static final double BOOST_DURATION = 0.4;                // 300 milliseconds boost duration
    
    // Indexor position settings
    public static final double INDEXOR_TICKS = 537.7/3;              // goBILDA 312 RPM motor: 120 degrees = 179 ticks
    
    // Shooter velocity control (ticks per second) - Blue alliance optimized
    public static double SHOOTER_TARGET_VELOCITY = 1500;      // Range: 1200-1800 ticks/sec (Blue back position)
    public static final double SHOOTER_SPEED_THRESHOLD = 0.95; // 95% of target speed
    public static final double SHOOTER_TICKS_PER_REVOLUTION = 1020.0; // goBILDA 435 RPM motor
    
    // Speed stabilization settings
    public static final double SHOOTER_SPEED_TOLERANCE = 25;    // ticks/sec tolerance for "stable" speed
    public static final double SHOOTER_STABILIZATION_TIME = 0.2; // Seconds to wait for speed stabilization (reduced from 0.3s)
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
    public static final double FORWARD_DISTANCE = 40.0;       // Distance to move sideways (inches)
    
    @Override
    public void runOpMode() {
        // Initialize motors and Road Runner drive
        initializeMotors();

        // Build all trajectory actions during initialization
        buildTrajectoryActions();
        
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
                .waitSeconds(0.3)  // Placeholder - replace with actual movement if needed
                .build();
        
        // Trajectory 2: Move 30 inches forward while turning to 130 degrees, then move 10 inches rearward with intake
        trajectory2 = drive.actionBuilder(START_POSE)
                .lineToXSplineHeading(START_POSE.position.x + 27.0, Math.toRadians(-115))
                .afterTime(0, (telemetryPacket) -> {
                    // Start indexor with velocity control - intake and conveyor already running
                    indexor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    indexor.setVelocity(INDEXOR_INTAKE_VELOCITY);
                    return false;
                })
                .setTangent(Math.toRadians(-115))
                .lineToY(START_POSE.position.y + 36.0) // 29.0

               .stopAndAdd((telemetryPacket) -> {
                    // Stop indexor only - keep intake and conveyor running
                    indexor.setVelocity(0);
                    return false;
                })
                .lineToYSplineHeading(START_POSE.position.y+1, Math.toRadians(0))
                .setTangent(Math.toRadians(0))
                .lineToX(START_POSE.position.x + 2.0)
                .build();

        // Trajectory 3: Move 30 inches forward while turning to 130 degrees, then move 10 inches rearward with intake
        trajectoryCloseOut = drive.actionBuilder(new Pose2d(START_POSE.position.x + 2.0,START_POSE.position.y+1,0))
                //.lineToXSplineHeading(START_POSE.position.x + 54.0, Math.toRadians(-115))
                .splineToLinearHeading(new Pose2d(START_POSE.position.x+49,START_POSE.position.y-5,Math.toRadians(-112)),0)
                .afterTime(0, (telemetryPacket) -> {
                    // Start indexor with velocity control - intake and conveyor already running
                    indexor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    indexor.setVelocity(INDEXOR_INTAKE_VELOCITY);
                    return false;
                })
                .setTangent(Math.toRadians(-112))
                .lineToY(START_POSE.position.y + 26.0) //18.0

                .stopAndAdd((telemetryPacket) -> {
                    // Stop indexor only - keep intake and conveyor running
                    indexor.setVelocity(0);
                    return false;
                })
                .lineToYSplineHeading(START_POSE.position.y+1, Math.toRadians(0))
                .setTangent(Math.toRadians(0))
                .lineToX(START_POSE.position.x + 2.0)
                .build();

            trajectoryCloseOutFinal = drive.actionBuilder(new Pose2d(START_POSE.position.x + 2.0,START_POSE.position.y+1,0))
                .lineToX(30)
                .build();
    }
    
    private void executeAutonomousSequence() {
        // Start shooter immediately - it will spin up during trajectory1 execution
        startShooterSystem();
        
        // Start intake and conveyor - keep them running throughout
        intake.setPower(AUTO_INDEXOR_POWER);
        conveyor.setPower(CONVEYOR_POWER);
        
        Actions.runBlocking(trajectory1);
        
        // Shooter should be ready or nearly ready by now
        waitForShooterSpeed();
        
        fireShot(1);
        moveIndexorToNextPosition();
        
        fireShot(2);
        moveIndexorToNextPosition();
        
        fireShot(3);
        moveIndexorToNextPosition();
        
        // Execute first part of trajectory 2 (forward movement with turn)
        Actions.runBlocking(trajectory2);
        ranTrajectory2 = true;
        // Diagnostic: Check indexer position after trajectory2
        int positionAfterTrajectory2 = indexor.getCurrentPosition();

        moveIndexorToNextPosition();
        fireShot(4);

        moveIndexorToNextPosition();

        fireShot(5);

        moveIndexorToNextPosition();

        fireShot(6);
        moveIndexorToNextPosition();

       Actions.runBlocking(trajectoryCloseOut);
        ranTrajectory2 = true;

        positionAfterTrajectory2 = indexor.getCurrentPosition();

        moveIndexorToNextPosition();
        fireShot(7);
        
        moveIndexorToNextPosition();

        fireShot(8);

        moveIndexorToNextPosition();

        Actions.runBlocking(trajectoryCloseOutFinal);

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
        ElapsedTime timeout = new ElapsedTime();
        timeout.reset();
        
        while (opModeIsActive() && timeout.seconds() < SHOOTER_SPINUP_TIMEOUT) {
            double currentVelocity = shooter.getVelocity();
            double speedPercentage = currentVelocity / SHOOTER_TARGET_VELOCITY;
            
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
        // Pre-fire velocity check and correction
        double preFire = shooter.getVelocity();
        double targetDifference = Math.abs(preFire - SHOOTER_TARGET_VELOCITY);
        
        if (targetDifference > SHOOTER_SPEED_TOLERANCE) {
            shooter.setVelocity(SHOOTER_TARGET_VELOCITY * SHOOTER_VELOCITY_CORRECTION_FACTOR);
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
        // Return trigger to home position
        triggerServo.setPosition(TRIGGER_HOME);
        
        // Wait between shots
        sleep((long)(WAIT_BETWEEN_SHOTS * 1000));
    }
    
    private void moveIndexorToNextPosition() {
        // Get current position and calculate target
        double currentPosition = indexor.getCurrentPosition();
        double targetPosition;
        
        if (ranTrajectory2) {
            double correction = currentPosition % INDEXOR_TICKS;
            targetPosition = currentPosition + INDEXOR_TICKS - correction;
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
        
        // Return trigger to home position
        triggerServo.setPosition(TRIGGER_HOME);
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
        triggerServo.setDirection(Servo.Direction.FORWARD);
        
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