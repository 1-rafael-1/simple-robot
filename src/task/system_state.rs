pub struct SystemState {
    pub operation_mode: OperationMode,
    pub battery_level: u8,
}

impl SystemState {
    pub fn new() -> Self {
        SystemState {
            operation_mode: OperationMode::Manual,
            battery_level: 100,
        }
    }
}

#[derive(Debug, Clone)]
pub enum OperationMode {
    Manual,
    Autonomous,
}
