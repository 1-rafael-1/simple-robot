// in task/handle_inertial_measurement.rs
use crate::system::resources::I2c0BusShared;
use defmt::*;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_time::{Duration, Timer};
// You'll need to add the mpu6500 driver crate to your dependencies

#[embassy_executor::task]
pub async fn inertial_measurement_handle(i2c_bus: &'static I2c0BusShared) {
    let _imu_i2c = I2cDevice::new(i2c_bus);

    // Initialize MPU6500
    // let mut mpu = Mpu6500::new(imu_i2c).await.unwrap();

    loop {
        Timer::after(Duration::from_millis(100)).await;
        info!("IMU reading");

        // Read sensor data
        // let accel = mpu.read_accel().await.unwrap();
        // let gyro = mpu.read_gyro().await.unwrap();

        // Process/use the data
        // info!("Accel: x={}, y={}, z={}", accel.x, accel.y, accel.z);
    }
}
