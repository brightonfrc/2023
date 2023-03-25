package frc.robot.dataStorageClasses;

public enum AutonomousSelection {
    // Fallback: individual components
    OldAutobalanceOnly,
    AutoBalanceOnly,
    // Basic: path to charge station; autobalance
    ClosestPathAndAutoBalance,
    MiddlePathAndAutoBalance,
    FurthestPathAndAutoBalance
}