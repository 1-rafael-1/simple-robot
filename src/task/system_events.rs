use crate::task::system_state::OperationMode;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;

pub static EVENT_CHANNEL: Channel<CriticalSectionRawMutex, Events, 10> = Channel::new();

pub async fn send_event(event: Events) {
    EVENT_CHANNEL.sender().send(event).await;
}

pub async fn wait_for_event() -> Events {
    EVENT_CHANNEL.receiver().receive().await
}

pub static SYSTEM_INDICATOR_CHANGED: Signal<CriticalSectionRawMutex, bool> = Signal::new();

pub async fn send_system_indicator_changed(value: bool) {
    SYSTEM_INDICATOR_CHANGED.signal(value);
}

pub async fn wait_for_system_indicator_changed() -> bool {
    SYSTEM_INDICATOR_CHANGED.wait().await
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
