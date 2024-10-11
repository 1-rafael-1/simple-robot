use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;

pub static SYSTEM_STATE: Mutex<CriticalSectionRawMutex, SystemState> = Mutex::new(SystemState {
    operation_mode: OperationMode::Manual,
    battery_level: 100,
    obstacle_detected: false,
});

pub struct SystemState {
    pub operation_mode: OperationMode,
    pub battery_level: u8,
    pub obstacle_detected: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub enum OperationMode {
    Manual,
    Autonomous,
}
