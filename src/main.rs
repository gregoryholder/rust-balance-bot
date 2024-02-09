#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

mod serial;
mod stepper;
mod timer;

use core::cell::{Cell, RefCell};

use arduino_hal::delay_ms;
use arduino_hal::hal::port::Dynamic;

use arduino_hal::port::{mode::Output, Pin};
use avr_device::atmega2560::TC1;
use avr_device::interrupt::Mutex;
use stepper::{RefStepperControl, Stepper};

#[allow(unused_imports)]
use arduino_hal::prelude::*;
use panic_halt as _;

use timer::millis_init;

type OutPin = Pin<Output, Dynamic>;

static LEFT_STEPPER: Mutex<RefCell<Option<Stepper<OutPin, OutPin, OutPin>>>> =
    Mutex::new(RefCell::new(None));

static RIGHT_STEPPER: Mutex<RefCell<Option<Stepper<OutPin, OutPin, OutPin>>>> =
    Mutex::new(RefCell::new(None));

const STEPPER_ISR_INTERVAL_US: u64 = 300;

// static mut SERIAL: Option<Usart> = None;

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    let serial = arduino_hal::default_serial!(dp, pins, 57600);
    serial::init(serial);

    let tc0 = dp.TC0;
    let tc1 = dp.TC1;

    millis_init(tc0);
    stepper_isr_init(tc1);

    // Enable interrupts globally

    let left_step_pin = pins.d30.into_output().downgrade();
    let left_dir_pin = pins.d31.into_output().downgrade();
    let left_en_pin = pins.d32.into_output().downgrade();

    let left = Stepper::new(left_en_pin, left_dir_pin, left_step_pin);

    avr_device::interrupt::free(|cs| {
        LEFT_STEPPER.borrow(cs).replace(Some(left));
    });

    let right_step_pin = pins.d40.into_output().downgrade();
    let right_dir_pin = pins.d41.into_output().downgrade();
    let right_en_pin = pins.d42.into_output().downgrade();

    let right = Stepper::new(right_en_pin, right_dir_pin, right_step_pin);

    avr_device::interrupt::free(|cs| {
        RIGHT_STEPPER.borrow(cs).replace(Some(right));
    });

    LEFT_STEPPER.set_acceleration(4000);
    LEFT_STEPPER.set_enable(true);

    RIGHT_STEPPER.set_acceleration(4000);
    RIGHT_STEPPER.set_enable(true);

    unsafe { avr_device::interrupt::enable() };

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

#[avr_device::interrupt(atmega2560)]
fn TIMER1_COMPA() {
    let cs = unsafe { avr_device::interrupt::CriticalSection::new() };

    let now_cell = STEPPER_NOW.borrow(cs);
    let now = now_cell.get();

    LEFT_STEPPER.run(now);
    RIGHT_STEPPER.run(now);

    now_cell.set(now + STEPPER_ISR_INTERVAL_US);
}
