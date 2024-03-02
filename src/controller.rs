// balancing robot main control logic

use core::default;

use embedded_hal::i2c::I2c;

use crate::{
    mpu6050::{ImuRead, Mpu6050, Mpu6050DataFrame},
    pid::PID,
    stepper::{RefStepperControl, StepperControl, StepperMutexControl},
};

enum SchedulerType {
    /// Always runs
    Always,

    /// Run periodically
    Periodic {
        period_ms: u32,
        last_ms: Option<u32>,
    },
}

impl SchedulerType {
    // if it's a periodic variant, if last_ms is None, set it to now_ms, if not increment it by period_ms
    // no need to check for is_late or skippable, that's the job of the Scheduler
    fn evaluate(&mut self, now_ms: u32) -> bool {
        match self {
            SchedulerType::Always => true,
            SchedulerType::Periodic { period_ms, last_ms } => {
                let Some(last_ms) = last_ms else {
                    *last_ms = Some(now_ms);
                    return true;
                };

                if now_ms - *last_ms > *period_ms {
                    *last_ms += *period_ms;
                    true
                } else {
                    false
                }
            }
        }
    }
}

struct Scheduler {
    scheduler_type: SchedulerType,
    skippable: bool,
}

impl Scheduler {
    fn new_periodic(period_ms: u32, skippable: bool) -> Self {
        Self {
            scheduler_type: SchedulerType::Periodic {
                period_ms,
                last_ms: None,
            },
            skippable,
        }
    }

    fn new_always(skippable: bool) -> Self {
        Self {
            scheduler_type: SchedulerType::Always,
            skippable,
        }
    }

    fn evaluate(&mut self, now_ms: u32, is_late: bool) -> bool {
        if is_late && self.skippable {
            false
        } else {
            self.scheduler_type.evaluate(now_ms)
        }
    }

    fn is_running_late(&self, now_ms: u32) -> bool {
        match &self.scheduler_type {
            SchedulerType::Always => false,
            SchedulerType::Periodic { period_ms, last_ms } => {
                if let Some(last_ms) = last_ms {
                    now_ms - *last_ms > 5 * *period_ms
                } else {
                    false
                }
            }
        }
    }
}

pub struct Controller<IMU, LEFT: 'static, RIGHT: 'static> {
    imu: IMU,
    left_stepper: &'static LEFT,
    right_stepper: &'static RIGHT,
    pid: PID,

    now_ms: u32,

    control_schedule: Scheduler,
    imu_read_schedule: Scheduler,

    pub pitch: i32,
    pub pitch_prior_raw: i32,

    pub accel_angle: i32,
    pub gyro: i32,
    pub gyro_pitch_prediction: i32,

    setpoint: i32,

    output: i32,
}

impl<IMU, LEFT, RIGHT> Controller<IMU, LEFT, RIGHT>
where
    IMU: ImuRead,
    LEFT: RefStepperControl,
    RIGHT: RefStepperControl,
{
    pub fn new(imu: IMU, left_stepper: &'static LEFT, right_stepper: &'static RIGHT) -> Self {
        left_stepper.set_acceleration(8000);
        right_stepper.set_acceleration(8000);

        left_stepper.set_enable(true);
        right_stepper.set_enable(true);

        Self {
            imu,
            left_stepper: &left_stepper,
            right_stepper: &right_stepper,
            pid: PID::new_with_constants(1, 0, 0, 300),
            now_ms: 0,
            pitch: 0,
            pitch_prior_raw: 0,
            accel_angle: 0,
            gyro: 0,
            setpoint: 0,
            output: 0,
            control_schedule: Scheduler::new_periodic(100, false),
            imu_read_schedule: Scheduler::new_periodic(200, false),

            gyro_pitch_prediction: 0,
            // imu_read_period: ControlMode::NotLate,
        }
    }

    fn read_imu(&mut self) -> Result<(i32, i32), IMU::Error> {
        let data = self.imu.get_data_frame()?;

        let accel_angle = data.calculate_pitch();
        let gyro = data.get_pitch_rate();

        Ok((accel_angle, gyro))
    }

    fn update_pitch(&mut self, delta_ms: u32, gyro: i32, accel_angle: Option<i32>) -> i32 {
        let gyro_angle_change = gyro * delta_ms as i32 / 1000;

        let gyro_pitch_prediction = self.pitch + gyro_angle_change;
        self.gyro_pitch_prediction = gyro_pitch_prediction;

        let pitch_raw = if let Some(accel_pitch_prediction) = accel_angle {
            // merge gyro with accelerometer (if available)
            (99 * gyro_pitch_prediction + 1 * accel_pitch_prediction) / 100
        } else {
            // else just use gyroscope integration
            gyro_pitch_prediction
        };

        self.pitch_prior_raw = pitch_raw;

        // filter the pitch using previous filtered and raw pitch, and new raw pitch
        // let pitch_filtered = (self.pitch + pitch_raw + self.pitch_prior_raw) / 3;

        self.pitch = pitch_raw;

        self.pitch
    }

    pub fn run(&mut self, delta_ms: u32) -> Result<(), IMU::Error> {
        self.now_ms += delta_ms;

        let is_late = self.is_late();

        let mut current_gyro = self.gyro;
        let mut current_accel_angle = None;

        if self.imu_read_schedule.evaluate(self.now_ms, is_late) {
            let (accel_angle, gyro) = self.read_imu()?;

            self.accel_angle = accel_angle;
            self.gyro = gyro;
            current_gyro = gyro;
            current_accel_angle = Some(accel_angle);
        }

        if self.control_schedule.evaluate(self.now_ms, is_late) {
            self.update_pitch(delta_ms, current_gyro, current_accel_angle);
        }

        Ok(())
    }

    pub fn get_pitch(&self) -> i32 {
        self.pitch
    }

    pub fn is_late(&self) -> bool {
        self.control_schedule.is_running_late(self.now_ms)
            || self.imu_read_schedule.is_running_late(self.now_ms)
    }
}

pub trait MpuHelper {
    const PITCH_OFFSET: i32 = -300;
    const GYRO_SENSITIVITY: i32 = 131;
    const GYRO_DRIFT: f32 = -1.95;
    const GYRO_DRIFT_SCALED: i32 = (Self::GYRO_DRIFT * 1000.0) as i32;

    fn calculate_pitch(&self) -> i32;

    fn get_pitch_rate(&self) -> i32;
}

impl MpuHelper for Mpu6050DataFrame {
    fn calculate_pitch(&self) -> i32 {
        let acc_x = self.accel_x as f32;
        let acc_z = self.accel_z as f32;
        (-fast_math::atan2(acc_x, acc_z) * 180.0 * 1000.0 / 3.14159) as i32 - Self::PITCH_OFFSET
    }

    fn get_pitch_rate(&self) -> i32 {
        self.gyro_y as i32 * 1000 / Self::GYRO_SENSITIVITY - Self::GYRO_DRIFT_SCALED
    }
}
