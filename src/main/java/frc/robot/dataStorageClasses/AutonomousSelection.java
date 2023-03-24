package frc.robot.dataStorageClasses;

public enum AutonomousSelection {
    // Fallback: individual components
    AutoBalanceOnlyForwards,
    AutoBalanceOnlyReverse,
    // Basic: path to charge station; autobalance
    ClosestPathAndAutoBalance,
    MiddlePathAndAutoBalance,
    FurthestPathAndAutoBalance
}