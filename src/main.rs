#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

mod mpu6050;
mod serial;
mod stepper;
mod timer;

use core::cell::Cell;

use arduino_hal::hal::port::Dynamic;
use arduino_hal::{delay_ms, delay_us};

use arduino_hal::port::{mode::Output, Pin};
use arrayvec::ArrayString;
use avr_device::atmega328p::TC1;
use avr_device::interrupt::Mutex;
// use mpu6050::device::{ACC_REGX_H, GYRO_REGX_H};
// use mpu6050::Mpu6050;
// use serial::Usart;
use stepper::{RefStepperControl, Stepper, StepperControl, StepperMutexControl};

#[allow(unused_imports)]
use arduino_hal::prelude::*;
use panic_halt as _;

use timer::{millis, millis_init};
use ufmt_float::uFmt_f32;

type OutPin = Pin<Output, Dynamic>;
type StepperType = Stepper<OutPin, OutPin, OutPin>;

static LEFT_STEPPER: StepperMutexControl<StepperType> = StepperMutexControl::new_empty();
static RIGHT_STEPPER: StepperMutexControl<StepperType> = StepperMutexControl::new_empty();

const STEPPER_ISR_INTERVAL_US: u64 = 500;

const GYRO_PERIOD_MS: u32 = 100;
const TS: f32 = GYRO_PERIOD_MS as f32 / 1000.0;

const GYRO_DRIFT: f32 = -1.9578;
const ALPHA: f32 = 0.05;

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    let mut serial = arduino_hal::default_serial!(dp, pins, 115200);
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
        loop {}
    }

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

    let mut pitch = 0.0;

    let mut str_buf = ArrayString::<16>::new();

    let mut k_p = 10.0;
    let mut k_i = 0.0;
    let mut k_d = 0.0;

    let mut k_bias = 0.0;

    let mut error_prior = 0.0;
    let mut integral = 0.0;

    let mut i = 0;

    let mut do_print = false;

    loop {
        let (acc_x, _, acc_z) = imu.get_accel().unwrap();

        let gyro_y = imu.get_gyro().unwrap().1 - GYRO_DRIFT;
        let acc_angle = -fast_math::atan2(acc_x, acc_z) * 180.0 / 3.14159;

        pitch = ALPHA * acc_angle + (1.0 - ALPHA) * (pitch + gyro_y * TS);

        let error = 0.0 - pitch;
        let error_squared = if error < 0.0 { -error } else { error } * error;
        
        integral += error_squared * TS;

        // let derivative = (error - error_prior) / TS;
        // error_prior = error;

        // base the derivative only on the gyro
        let derivative = gyro_y;


        let output = k_p * error_squared + k_i * integral + k_d * derivative + k_bias;

        LEFT_STEPPER.set_speed(output as i32);
        RIGHT_STEPPER.set_speed(-output as i32);

        while millis() - now < GYRO_PERIOD_MS {
            // if a character is available, read it, and then read until newline
            if let Ok(c) = serial.read() {
                str_buf.push(c as char);

                loop {
                    if let Ok(c) = serial.read() {
                        let c = c as char;

                        if c == '\n' {
                            break;
                        }

                        str_buf.push(c);
                    }
                }

                let mut chars = str_buf.chars();
                let cmnd = chars.next().unwrap();

                if let Ok(val) = chars.as_str().parse::<i32>() {
                    let val_f = (val as f32) / 100.0;
                    match cmnd {
                        'p' => k_p = val_f * 100.0,
                        'i' => {
                            // When changing the integral, change the accumulated integral in a way that
                            // avoids a sudden jump in the output.

                            // example:
                            // k_i = 1, integral = 100 -> k_i * integral = 100
                            // k_i = 2, integral = 100 -> k_i * integral = 200 -> sudden jump
                            // k_i = 2, integral = 100 -> integral = 100 / new_k_i * old_k_i = 50 -> k_i * integral = 100 -> no sudden jump

                            let new_k_i = val_f * 100.0;

                            if new_k_i == 0.0 {
                                integral = 0.0;
                            } else {
                                integral = integral * k_i / new_k_i;
                            }
                            k_i = new_k_i;
                        }
                        'd' => k_d = val_f,
                        'b' => k_bias = val_f * 100.0,
                        'v' => do_print = val == 1,
                        _ => {}
                    }
                }

                let p = uFmt_f32::Two(k_p);
                let i = uFmt_f32::Two(k_i);
                let d = uFmt_f32::Two(k_d);
                let bias = uFmt_f32::Two(k_bias);

                let integral = uFmt_f32::Two(integral);

                ufmt::uwriteln!(
                    &mut serial,
                    "p: {}, i: {}, d: {}, b: {}, integral: {}\n",
                    p,
                    i,
                    d,
                    bias,
                    integral
                )
                .unwrap_infallible();

                str_buf.clear();
            }
        }

        i = i % 2;

        if i == 0 && do_print {
            // multiply by 100 to get 2 decimal places
            let pitch = (pitch * 100.0) as i32;
            let integral = (integral * 100.0) as i32;
            let derivative = (derivative * 100.0) as i32;
            let output = (output * 100.0) as i32;

            ufmt::uwriteln!(
                &mut serial,
                // "pitch: {}, error: {}, integral: {}, derivative: {}, output: {}, p: {}, i: {}, d: {}, bias: {}",
                "{},{},{},{},{}",
                millis(),
                pitch,
                integral,
                derivative,
                output // p,
                       // i,
                       // d,
                       // bias,
            )
            .unwrap_infallible();
        }

        i += 1;
        now += GYRO_PERIOD_MS;
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
