use crate::task::system_state::OperationMode;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch;

pub static EVENT_WATCH: watch::Watch<CriticalSectionRawMutex, Events, 1> = watch::Watch::new();

pub fn send_event(event: Events) {
    let sender = EVENT_WATCH.sender();
    sender.send(event);
}

#[derive(Debug, Clone)]
pub enum Events {
    ModeChange(OperationMode),
    ObstacleDetected(bool),
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
