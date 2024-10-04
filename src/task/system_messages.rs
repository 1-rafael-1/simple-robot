#[derive(Debug, Clone)]
pub enum SystemMessage {
    MotorCommand(DriveCommand),
    // SensorReading(SensorReading),
    // ModeChange(OperationMode),
}

#[derive(Debug, Clone)]
pub enum DriveCommand {
    Left(u32),
    Right(u32),
    Forward(u8),
    Backward(u8),
    Brake,
    Coast,
    Standby,
}
