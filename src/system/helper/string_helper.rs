//! Helper functions for string manipulation and conversion.

/// Converts a &str to a `heapless::String` with a maximum capacity of 20 characters.
/// If the input string exceeds this length, a warning is logged and None is returned.
pub fn status_text(text: &str) -> Option<heapless::String<20>> {
    heapless::String::try_from(text).map_or_else(
        |_| {
            defmt::warn!("Calibration status text too long: {}", text);
            None
        },
        Some,
    )
}
