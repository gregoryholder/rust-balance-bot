#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]


mod serial;
mod mpu6050;
mod stepper;
mod timer;

use core::cell::Cell;

use arduino_hal::delay_ms;
use arduino_hal::hal::port::Dynamic;

use arduino_hal::port::{mode::Output, Pin};
use avr_device::atmega328p::{ac, TC1};
use avr_device::interrupt::Mutex;
// use mpu6050::device::{ACC_REGX_H, GYRO_REGX_H};
// use mpu6050::Mpu6050;
// use serial::Usart;
use stepper::{RefStepperControl, Stepper, StepperControl, StepperMutexControl};

use arduino_hal::prelude::*;
use panic_halt as _;

use timer::{millis, millis_init};
use ufmt_float::uFmt_f32;

type OutPin = Pin<Output, Dynamic>;
type StepperType = Stepper<OutPin, OutPin, OutPin>;

static LEFT_STEPPER: StepperMutexControl<StepperType> = StepperMutexControl::new_empty();
static RIGHT_STEPPER: StepperMutexControl<StepperType> = StepperMutexControl::new_empty();

const STEPPER_ISR_INTERVAL_US: u64 = 500;

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    let mut serial = arduino_hal::default_serial!(dp, pins, 57600);
    // serial::init(serial);

    let i2c = arduino_hal::I2c::new(
        dp.TWI,
        pins.a4.into_pull_up_input(),
        pins.a5.into_pull_up_input(),
        50000,
    );
    let mut delay = arduino_hal::Delay::new();

    let mut imu = mpu6050::Mpu6050::new(i2c);

    if let Err(_) = imu.init(&mut delay) {
        // ufmt::uwriteln!(&mut serial, "Failed to initialize IMU").unwrap();
        loop {}
    }

    // ufmt::uwriteln!(&mut serial, "IMU initialized").unwrap();

    // let mut imu = Mpu6050::new(i2c);
    // imu.init(&mut delay).unwrap();

    let tc0 = dp.TC0;
    let tc1 = dp.TC1;

    millis_init(tc0);
    stepper_isr_init(tc1);

    let left_dir_pin = pins.d2.into_output().downgrade();
    let left_step_pin = pins.d3.into_output().downgrade();
    let left_en_pin = pins.d4.into_output().downgrade();

    let left = Stepper::new(left_en_pin, left_dir_pin, left_step_pin);

    LEFT_STEPPER.assign(left);

    let right_dir_pin = pins.d5.into_output().downgrade();
    let right_step_pin = pins.d6.into_output().downgrade();
    let right_en_pin = pins.d7.into_output().downgrade();

    let right = Stepper::new(right_en_pin, right_dir_pin, right_step_pin);

    RIGHT_STEPPER.assign(right);

    LEFT_STEPPER.set_acceleration(8000);
    LEFT_STEPPER.set_enable(true);

    RIGHT_STEPPER.set_acceleration(8000);
    RIGHT_STEPPER.set_enable(true);

    // Enable interrupts globally
    unsafe { avr_device::interrupt::enable() };

    let mut now = 0;

    const GYRO_PERIOD: u32 = 50;
    const TS: f32 = GYRO_PERIOD as f32 / 1000.0;
    
    const GYRO_DRIFT: f32 = -1.9578;

    let mut pitch = 0.0;

    loop {
        let (acc_x, _, acc_z) = imu.get_accel().unwrap();

        let gyro_y = imu.get_gyro().unwrap().1 - GYRO_DRIFT;

        let acc_angle = -fast_math::atan2(acc_x, acc_z) * 180.0 / 3.14159;

        const ALPHA: f32 = 0.1;
        pitch = ALPHA * acc_angle + (1.0 - ALPHA) * (pitch + gyro_y * TS);

        let pitch_int = (pitch * 100.0) as i32;

        LEFT_STEPPER.set_speed( -pitch_int);
        RIGHT_STEPPER.set_speed(pitch_int);

        while millis() - now < GYRO_PERIOD {
            delay_ms(1);
        }
        now += GYRO_PERIOD;
    }
}

fn stepper_isr_set_interval(tc1: &mut TC1, interval_us: u64) {
    const PRESCALER: u64 = 1;

    let ticks_per_interval = 16 * interval_us / PRESCALER;

    tc1.ocr1a.write(|w| w.bits(ticks_per_interval as u16));
}

fn stepper_isr_init(mut tc1: TC1) {
    stepper_isr_set_interval(&mut tc1, STEPPER_ISR_INTERVAL_US);

    tc1.tccr1a.write(|w| w.wgm1().bits(0b00));
    tc1.tccr1b.write(|w| {
        w.wgm1().bits(0b01);
        w.cs1().direct()
    });

    tc1.timsk1.write(|w| w.ocie1a().set_bit());
}

static STEPPER_NOW: Mutex<Cell<u64>> = Mutex::new(Cell::new(0));

#[avr_device::interrupt(atmega328p)]
fn TIMER1_COMPA() {
    let cs = unsafe { avr_device::interrupt::CriticalSection::new() };

    let now_cell = STEPPER_NOW.borrow(cs);
    let now = now_cell.get();

    let mut left_stepper_ref = LEFT_STEPPER.0.borrow(cs).borrow_mut();
    let mut right_stepper_ref = RIGHT_STEPPER.0.borrow(cs).borrow_mut();

    if let Some(left_stepper) = left_stepper_ref.as_mut() {
        left_stepper.run(now);
    }

    if let Some(right_stepper) = right_stepper_ref.as_mut() {
        right_stepper.run(now);
    }

    now_cell.set(now + STEPPER_ISR_INTERVAL_US);
}
