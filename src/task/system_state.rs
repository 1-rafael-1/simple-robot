use super::system_messages::OperationMode;

pub struct SystemState {
    pub operation_mode: OperationMode,
    pub current_distance: u32,
    pub is_obstacle_detected: bool,
    pub battery_level: u8,
}

impl SystemState {
    pub fn new() -> Self {
        SystemState {
            operation_mode: OperationMode::Manual,
            current_distance: 0,
            is_obstacle_detected: false,
            battery_level: 100,
        }
    }

    pub fn update_distance(&mut self, distance: u32) {
        self.current_distance = distance;
        self.is_obstacle_detected = distance < 20; // Example threshold
    }
}
