#[derive(Debug, Clone)]
pub enum SystemMessage {
    MotorCommand(MotorCommand),
    SensorReading(SensorReading),
    ModeChange(OperationMode),
}

#[derive(Debug, Clone)]
pub enum MotorCommand {
    Forward,
    Backward,
    Left,
    Right,
    Stop,
}

#[derive(Debug, Clone)]
pub struct SensorReading {
    pub distance: u32,
}

#[derive(Debug, Clone)]
pub enum OperationMode {
    Manual,
    Autonomous,
}
