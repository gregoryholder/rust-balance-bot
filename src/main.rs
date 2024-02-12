#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

mod serial;
mod stepper;
mod timer;

use core::cell::Cell;

use arduino_hal::delay_ms;
use arduino_hal::hal::port::Dynamic;

use arduino_hal::port::{mode::Output, Pin};
use avr_device::atmega328p::TC1;
use avr_device::interrupt::Mutex;
use mpu6050::Mpu6050;
use serial::Usart;
use stepper::{RefStepperControl, Stepper, StepperMutexControl};

#[allow(unused_imports)]
use arduino_hal::prelude::*;
use panic_halt as _;

use timer::{millis, millis_init};

type OutPin = Pin<Output, Dynamic>;
type StepperType = Stepper<OutPin, OutPin, OutPin>;

static LEFT_STEPPER: StepperMutexControl<StepperType> = StepperMutexControl::new_empty();

static RIGHT_STEPPER: StepperMutexControl<StepperType> = StepperMutexControl::new_empty();

const STEPPER_ISR_INTERVAL_US: u64 = 300;

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
    let mut imu = Mpu6050::new(i2c);

    let mut delay = arduino_hal::Delay::new();

    let _ = imu.init(&mut delay);

    let tc0 = dp.TC0;
    let tc1 = dp.TC1;

    millis_init(tc0);
    stepper_isr_init(tc1);

    // Enable interrupts globally

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

    unsafe { avr_device::interrupt::enable() };

    let mut now = 0;

    const GYRO_PERIOD: u32 = 20;

    loop {
        // let acc = gyro.get_acc_angles();
        let gyro = imu.get_gyro().unwrap();

        // let x = (gyro.x * 1000.0) as i32;
        // let y = (gyro.y * 1000.0) as i32;
        // let z = (gyro.z * 1000.0) as i32;

        // let x_gyro = uFmt_f32::Two(gyro.x);
        // let y_gyro = uFmt_f32::Two(gyro.y);
        // let z_gyro = uFmt_f32::Two(gyro.z);

        // println!("{}ms\t| x: \t{}, y: \t{}, z: \t{}", now, x_gyro, y_gyro, z_gyro);

        // println!("{}, {}, {}"x_gyro, x_gyro, y_gyro, z_gyro);

        // ufmt::uWrite(&mut serial::GLOBAL_SERIAL.borrow_mut(), "x: ").unwrap();

        // write raw floats bytes to serial

        write_float(&mut serial, gyro.x);
        // write_float(&mut serial, gyro.y);
        // write_float(&mut serial, gyro.z);

        ufmt::uwrite!(serial, "\n").unwrap();

        // let z = acc.z;

        // let x = uFmt_f32::Two(x);
        // let y = uFmt_f32::Two(y);

        // println!("{} {}", x, y);

        // scale and convert to ints
        // let x = (x * 2000.0) as i32;
        // let y = (y * 2000.0) as i32;
        // let z = (z * 100.0) as i32;

        // print the values
        // println!("x: {}, y: {}", x, y);

        // LEFT_STEPPER.set_speed(x);
        // RIGHT_STEPPER.set_speed(y);

        while millis() - now < GYRO_PERIOD {
            delay_ms(1);
        }
        now += GYRO_PERIOD;
    }

    #[allow(unreachable_code)]
    loop {
        LEFT_STEPPER.set_speed(1000);
        RIGHT_STEPPER.set_speed(1500);
        delay_ms(1000);

        LEFT_STEPPER.set_speed(2000);
        RIGHT_STEPPER.set_speed(200);
        delay_ms(1000);

        LEFT_STEPPER.set_speed(1300);
        RIGHT_STEPPER.set_speed(1000);
        delay_ms(1000);

        LEFT_STEPPER.set_speed(-500);
        RIGHT_STEPPER.set_speed(500);
        delay_ms(1000);

        LEFT_STEPPER.set_speed(-1000);
        RIGHT_STEPPER.set_speed(-1000);
        delay_ms(2000);

        LEFT_STEPPER.set_enable(true);
        RIGHT_STEPPER.set_enable(false);
        delay_ms(2000);

        LEFT_STEPPER.set_enable(true);
        RIGHT_STEPPER.set_enable(true);
    }
}

fn write_float(serial: &mut Usart, f: f32) {
    let bytes = f.to_le_bytes();
    for byte in bytes.iter() {
        serial.write_byte(*byte);
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

    LEFT_STEPPER.run(now);
    RIGHT_STEPPER.run(now);

    now_cell.set(now + STEPPER_ISR_INTERVAL_US);
}
