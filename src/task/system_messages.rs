use crate::task::system_state::OperationMode;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

// Adjust the channel size as needed
pub static EVENT_CHANNEL: Channel<CriticalSectionRawMutex, Events, 10> = Channel::new();

pub async fn send_event(event: Events) {
    EVENT_CHANNEL.sender().send(event).await;
}

#[derive(Debug, Clone)]
pub enum Events {
    ModeSet(OperationMode),
    ObstacleDetected(bool),
    BatteryLevelMeasured(u8),
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
