#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

mod controller;
mod mpu6050;
mod pid;
mod serial;
mod stepper;
mod timer;

use core::cell::Cell;
use core::cmp::{max, min};

use arduino_hal::hal::port::{Dynamic, PD2, PD3, PD4, PD5, PD6, PD7};
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
// use panic_serial as _;

use timer::{millis, millis_init};

use crate::controller::{Controller, MpuHelper};
use crate::mpu6050::{ImuRead, Mpu6050DataFrame};

type OutPin<PIN> = Pin<Output, PIN>;
type StepperType<E, D, S> = Stepper<OutPin<E>, OutPin<D>, OutPin<S>>;

//
static LEFT_STEPPER: StepperMutexControl<StepperType<PD4, PD2, PD3>> =
    StepperMutexControl::new_empty();
static RIGHT_STEPPER: StepperMutexControl<StepperType<PD7, PD5, PD6>> =
    StepperMutexControl::new_empty();

const STEPPER_ISR_INTERVAL_US: u16 = 500;
const MAIN_LOOP_INTERVAL_MS: u16 = 2;
const GYRO_READ_INTERVAL_MS: u16 = 15;
const SERIAL_INTERVAL_MS: u16 = 100;

#[derive(Debug, PartialEq, Clone, Copy)]
enum ControlState {
    Booting,
    Ready,
    Stabilizing,
    Stable,
}

// panic_serial::impl_panic_handler!(
//   // This is the type of the UART port to use for printing the message:
//   arduino_hal::usart::Usart<
//     arduino_hal::pac::USART0,
//     arduino_hal::port::Pin<arduino_hal::port::mode::Input, arduino_hal::hal::port::PD0>,
//     arduino_hal::port::Pin<arduino_hal::port::mode::Output, arduino_hal::hal::port::PD1>
//   >
// );

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    let mut serial = arduino_hal::default_serial!(dp, pins, 57600);
    // let serial = share_serial_port_with_panic(serial);

    // serial::init(&mut serial);

    let i2c = arduino_hal::I2c::new(
        dp.TWI,
        pins.a4.into_pull_up_input(),
        pins.a5.into_pull_up_input(),
        200000,
    );
    let mut delay = arduino_hal::Delay::new();
    let mut imu = mpu6050::Mpu6050::new(i2c);

    imu.init(&mut delay).expect("Error initializing MPU6050");

    imu.set_dlpf_cfg(mpu6050::DlpfCfg::Mode6).unwrap();

    let tc0 = dp.TC0;
    let tc1 = dp.TC1;

    millis_init(tc0);
    stepper_isr_init(tc1);

    let left_en_pin = pins.d4.into_output();
    let left_dir_pin = pins.d2.into_output();
    let left_step_pin = pins.d3.into_output();

    let left = Stepper::new(left_en_pin, left_dir_pin, left_step_pin);

    LEFT_STEPPER.assign(left);

    let right_en_pin = pins.d7.into_output();
    let right_dir_pin = pins.d5.into_output();
    let right_step_pin = pins.d6.into_output();

    let right = Stepper::new(right_en_pin, right_dir_pin, right_step_pin);

    RIGHT_STEPPER.assign(right);

    LEFT_STEPPER.set_acceleration(8000);
    LEFT_STEPPER.set_enable(true);

    RIGHT_STEPPER.set_acceleration(8000);
    RIGHT_STEPPER.set_enable(true);

    // let mut controller: Controller<_, _, _> =
    //     controller::Controller::new(imu, &LEFT_STEPPER, &RIGHT_STEPPER);

    // Enable interrupts globally
    unsafe { avr_device::interrupt::enable() };

    // scaled by 100
    let mut pitch: i32 = 0;

    let mut controller = pid::PID::new_with_constants(6, 10, 6, 1000);

    let mut error_prior_raw = 0;
    let mut error_prior_filtered = 0;

    let mut accel_angle = 0;
    let mut gyro: i32 = 0;

    let mut setpoint = 500;

    let mut status = ControlState::Booting;

    let mut saved_pid = (0, 0, 0);

    let mut output: i32 = 0;

    let mut loop_counter = 0;

    const IMU_READ_COUNT: u16 = GYRO_READ_INTERVAL_MS / MAIN_LOOP_INTERVAL_MS;
    let mut now = millis();

    let mut serial_prints_skipped = 0;

    // loop {
    //     let mut delta_ms = millis() - now;
    //     while delta_ms == 0 {
    //         delta_ms = millis() - now;
    //     }

    //     now += delta_ms;

    //     controller.run(delta_ms).unwrap();

    //     let Controller {
    //         pitch,
    //         accel_angle,
    //         gyro,
    //         gyro_pitch_prediction,
    //         ..
    //     } = controller;

    //     if controller.is_late() {
    //         serial_prints_skipped += 1;
    //         if serial_prints_skipped % 500 != 0 {
    //             continue;
    //         }
    //     }

    //     // print pitch, accel_angle, gyro
    //     println!(
    //         "({},{},{},{}),{},{}"
    //         pitch,
    //         accel_angle,
    //         gyro,
    //         gyro_pitch_prediction,
    //         serial_prints_skipped,
    //         now
    //     );

    //     // delay_ms(10);
    // }

        loop {
            let loop_is_late = millis() - now > MAIN_LOOP_INTERVAL_MS as u32;

            if !loop_is_late {
                let mpu_data = imu.get_data_frame().unwrap();

                accel_angle = mpu_data.calculate_pitch();
                gyro = mpu_data.get_pitch_rate();

                const GYRO_WEIGHT: i32 = 999;
                const FUSION_TOTAL: i32 = 1000;

                let gyro_angle_change = gyro * MAIN_LOOP_INTERVAL_MS as i32 / 1000;

                if status == ControlState::Booting {
                    // value the accel value more than during normal operation (1 to 9)
                    pitch = (1 * accel_angle + 1 * (pitch + gyro_angle_change)) / 2;
                } else {
                    // only combine the gyro and accel values if the imu has been read recently
                    pitch = ((FUSION_TOTAL - GYRO_WEIGHT) * accel_angle
                        + GYRO_WEIGHT * (pitch + gyro_angle_change))
                        / FUSION_TOTAL;
                }
            } else {
                // else just use the gyro value
                pitch += gyro * MAIN_LOOP_INTERVAL_MS as i32 / 1000;
            }

            // basic sensor fusion

            let error_raw = setpoint - pitch;

            const ALPHA: i32 = 6;

            // filter the error using previous values
            let error = (error_prior_filtered * ALPHA + error_raw + error_prior_raw) / (ALPHA + 2);

            error_prior_raw = error_raw;
            error_prior_filtered = error;

            match (status, millis(), error.abs(), output.abs()) {
                (ControlState::Booting, 1000.., _, _) => {
                    status = ControlState::Ready;
                    saved_pid = (controller.get_p(), controller.get_i(), controller.get_d());

                    ufmt::uwriteln!(&mut serial, "Ready").unwrap_infallible();
                }

                (ControlState::Stable, _, 20_000.., _) => {
                    status = ControlState::Ready;
                    saved_pid = (controller.get_p(), controller.get_i(), controller.get_d());

                    ufmt::uwriteln!(&mut serial, "Fallen !!").unwrap_infallible();
                }

                (ControlState::Stabilizing, _, 20_000.., _) => {
                    status = ControlState::Ready;

                    ufmt::uwriteln!(&mut serial, "Stabilizing failed").unwrap_infallible();
                }

                (ControlState::Ready, _, ..=5_000, _) => {
                    // controller.set_p(2);
                    // controller.set_i(0);
                    // controller.set_d(4);

                    controller.clear_integral();
                    status = ControlState::Stabilizing;
                    setpoint = 0;
                    ufmt::uwriteln!(&mut serial, "Stabilizing").unwrap_infallible();
                }

                // (ControlState::Ready, _, ..=20_000, _) => {
                //     controller.set_p(0);
                //     controller.set_i(0);
                //     controller.set_d(4);
                // }
                // (ControlState::Ready, _, _, _) => {
                //     controller.set_p(0);
                //     controller.set_i(0);
                //     controller.set_d(0);
                // }
                (ControlState::Stabilizing, _, ..=5_000, ..=20_000) => {
                    // restore the saved pid values
                    controller.set_p(saved_pid.0);
                    controller.set_i(saved_pid.1);
                    controller.set_d(saved_pid.2);
                    status = ControlState::Stable;
                    setpoint = 0;
                    ufmt::uwriteln!(&mut serial, "Stable").unwrap_infallible();
                }
                _ => {}
            }

            controller.update(error, MAIN_LOOP_INTERVAL_MS as u32);

            output = (output * 1 + controller.get_output() * 9) / 10;

            // if state is booting or ready, don't move
            if status == ControlState::Booting || status == ControlState::Ready {
                output = 0;
            }

            // else if output > 0 {
            //     setpoint += 5;
            // } else if output < 0 {
            //     setpoint -= 5;
            // }

            let mut current_speed = 0;
            let mut delta_step = 0;

            avr_device::interrupt::free(|cs| {
                let mut left_stepper_ref = LEFT_STEPPER.0.borrow(cs).borrow_mut();
                let mut right_stepper_ref = RIGHT_STEPPER.0.borrow(cs).borrow_mut();

                let left_stepper = left_stepper_ref.as_mut().unwrap();
                let right_stepper = right_stepper_ref.as_mut().unwrap();

                current_speed = left_stepper.current_speed;
                delta_step = left_stepper.delta_step;

                if output.abs() > 5_000 {
                    left_stepper.set_speed(output);
                    right_stepper.set_speed(-output);
                } else {
                    left_stepper.set_speed(0);
                    right_stepper.set_speed(0);
                }
            });

            now += MAIN_LOOP_INTERVAL_MS as u32;
        }
}

fn stepper_isr_set_interval(tc1: &mut TC1, interval_us: u16) {
    const PRESCALER: u16 = 8;

    let ticks_per_interval = 16 * interval_us / PRESCALER - 1;

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

// static STEPPER_NOW: Mutex<Cell<u64>> = Mutex::new(Cell::new(0));

// static ISR_COUNT: Mutex<Cell<u16>> = Mutex::new(Cell::new(0));

// static ISR_LAST_MS: Mutex<Cell<u32>> = Mutex::new(Cell::new(0));

static ISR_COUNT: Mutex<Cell<u16>> = Mutex::new(Cell::new(0));

const STEPPER_UPDATE_INTERVAL_MS: u32 = 1;
const ISR_COUNT_MAX: u16 =
    (STEPPER_UPDATE_INTERVAL_MS * 1000 / STEPPER_ISR_INTERVAL_US as u32) as u16;

#[avr_device::interrupt(atmega328p)]
fn TIMER1_COMPA() {
    let cs = unsafe { avr_device::interrupt::CriticalSection::new() };
    // let now_cell = STEPPER_NOW.borrow(cs);
    // let now = now_cell.get();

    let mut left_stepper_ref = LEFT_STEPPER.0.borrow(cs).borrow_mut();
    let mut right_stepper_ref = RIGHT_STEPPER.0.borrow(cs).borrow_mut();

    let Some(left_stepper) = left_stepper_ref.as_mut() else {
        return;
    };

    let Some(right_stepper) = right_stepper_ref.as_mut() else {
        return;
    };

    left_stepper.do_step();
    right_stepper.do_step();

    // increment the count

    let count_cell = ISR_COUNT.borrow(cs);
    let count = count_cell.get();
    count_cell.set(count + 1);

    // if the count is 100, reset the count and update the steppers
    if count == ISR_COUNT_MAX {
        count_cell.set(0);
        left_stepper.update(STEPPER_UPDATE_INTERVAL_MS);
        right_stepper.update(STEPPER_UPDATE_INTERVAL_MS);
    }

    // let count_cell = ISR_COUNT.borrow(cs);
    // let count = count_cell.get();
    // count_cell.set(count + 1);

    // now_cell.set(now + STEPPER_ISR_INTERVAL_US);
}
