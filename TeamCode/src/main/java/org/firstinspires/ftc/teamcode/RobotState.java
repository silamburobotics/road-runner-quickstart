package org.firstinspires.ftc.teamcode;

/**
 * RobotState - Persistent storage class for robot state across OpModes
 * 
 * This static class maintains robot state information that needs to persist
 * from Autonomous to TeleOp, particularly the indexer position.
 * 
 * Using a static class ensures the data survives the transition between
 * OpModes within the same robot controller session.
 */
public class RobotState {
    
    // Indexer position preservation
    private static double savedIndexerPosition = 0.0;
    private static boolean indexerPositionSaved = false;
    
    // Timestamp for debugging
    private static long saveTimestamp = 0;
    
    /**
     * Save the current indexer position from Autonomous
     * Call this at the end of autonomous OpMode
     * 
     * @param position The indexer position in ticks to save
     */
    public static void saveIndexerPosition(double position) {
        savedIndexerPosition = position;
        indexerPositionSaved = true;
        saveTimestamp = System.currentTimeMillis();
    }
    
    /**
     * Retrieve the saved indexer position for TeleOp
     * Call this at the start of TeleOp to restore position
     * 
     * @return The saved indexer position in ticks, or 0.0 if none saved
     */
    public static double getSavedIndexerPosition() {
        return savedIndexerPosition;
    }
    
    /**
     * Check if an indexer position has been saved
     * 
     * @return true if position was saved in autonomous, false otherwise
     */
    public static boolean hasIndexerPositionSaved() {
        return indexerPositionSaved;
    }
    
    /**
     * Get the timestamp when position was saved
     * Useful for debugging and telemetry
     * 
     * @return System time in milliseconds when position was saved
     */
    public static long getSaveTimestamp() {
        return saveTimestamp;
    }
    
    /**
     * Clear the saved indexer position
     * Useful for testing or when starting fresh
     */
    public static void clearIndexerPosition() {
        savedIndexerPosition = 0.0;
        indexerPositionSaved = false;
        saveTimestamp = 0;
    }
    
    /**
     * Reset all robot state
     * Call this to clear all persistent data
     */
    public static void resetAll() {
        clearIndexerPosition();
    }
}
