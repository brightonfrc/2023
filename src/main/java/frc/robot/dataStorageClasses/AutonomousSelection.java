package frc.robot.dataStorageClasses;

public enum AutonomousSelection {
    // Fallback: individual components
    AutoBalanceOnly,
    // Basic: path to charge station; autobalance
    ClosestPathAndAutoBalance,
    MiddlePathAndAutoBalance,
    FurthestPathAndAutoBalance, 
    // Average: exit community; autobalance
    ClosestExitCommunityAndAutoBalance, 
    FurthestExitCommunityAndAutoBalance
}