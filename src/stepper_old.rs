use core::mem::{self, MaybeUninit};

use arduino_hal::port::{mode::Output, Pin};
use avr_device::atmega2560::{tc1::tccr1b::CS1_A, TC1};
use embedded_hal::digital::OutputPin;

struct InterruptState {
    blinker: Pin<Output>,
    countdown: u32,
}

pub static mut TARGET_SPEED: u32 = 1000;
pub static mut INTERRUPT_STATE: MaybeUninit<InterruptState> = mem::MaybeUninit::uninit();

pub enum Direction {
    Forward,
    Backward,
}

pub struct Stepper<E, D, S> {
    enable_pin: E,
    direction_pin: D,
    step_pin: S,
    target_acceleration: f32,
    target_speed: f32,
}

impl<E, D, S> Stepper<E, D, S>
where
    E: OutputPin,
    D: OutputPin,
    S: OutputPin,
{
    pub fn new(enable_pin: E, direction_pin: D, step_pin: S) -> Self {
        Stepper {
            enable_pin,
            direction_pin,
            step_pin,
        }
    }

    pub fn set_acceleration(&mut self, acc: f32) {
        self.target_acceleration = acc;
    }

    pub fn set_speed(&mut self, speed: f32) {
        self.target_speed = speed;
    }

    pub fn step(&mut self) {
        self.step_pin.set_high();
        
    }

    pub fn run(&mut self) {

    }
}

const fn calc_overflow(clock_hz: u32, target_hz: u32, prescale: u32) -> u32 {
    /*
    https://github.com/Rahix/avr-hal/issues/75
    reversing the formula F = 16 MHz / (256 * (1 + 15624)) = 4 Hz
     */
    clock_hz / target_hz / prescale - 1
}

pub fn stepper_controller_setup(tmr1: &TC1, target_hz: u32) -> Result<(), &str>{
    
    use arduino_hal::clock::Clock;

    const CLOCK_FREQUENCY_HZ: u32 = arduino_hal::DefaultClock::FREQ;
    // const CLOCK_SOURCE: CS1_A = CS1_A::PRESCALE_256;
    const CLOCK_SOURCE: CS1_A = CS1_A::PRESCALE_8;
    
    let clock_divisor: u32 = match CLOCK_SOURCE {
        CS1_A::DIRECT => 1,
        CS1_A::PRESCALE_8 => 8,
        CS1_A::PRESCALE_64 => 64,
        CS1_A::PRESCALE_256 => 256,
        CS1_A::PRESCALE_1024 => 1024,
        CS1_A::NO_CLOCK | CS1_A::EXT_FALLING | CS1_A::EXT_RISING => {
            return Err("uhoh, code tried to set the clock source to something other than a static prescaler");
        }
    };

    // how many ticks per cycle
    let ticks = calc_overflow(CLOCK_FREQUENCY_HZ, target_hz, clock_divisor) as u16;


    tmr1.tccr1a.write(|w| w.wgm1().bits(0b00));
    tmr1.tccr1b.write(|w| {
        w.cs1()
            //.prescale_256()
            .variant(CLOCK_SOURCE)
            .wgm1()
            .bits(0b01)
    });
    tmr1.ocr1a.write(|w| w.bits(ticks));
    tmr1.timsk1.write(|w| w.ocie1a().set_bit()); //enable this specific interrupt
    

    Ok(())
}

pub fn set_pin(pin: Pin<Output>) {
    unsafe {
        INTERRUPT_STATE = mem::MaybeUninit::new(InterruptState {
            blinker: pin,
            countdown: 100,
        });
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
        avr_device::interrupt::enable();
    }


}

#[avr_device::interrupt(atmega2560)]
fn TIMER1_COMPA() {
    let state = unsafe {
        // SAFETY: We _know_ that interrupts will only be enabled after the LED global was
        // initialized so this ISR will never run when LED is uninitialized.
        &mut *INTERRUPT_STATE.as_mut_ptr()
    };

    if state.countdown == 0 {
        unsafe {state.countdown = 2001 - TARGET_SPEED};
        state.blinker.toggle();
    }

    state.countdown -= 1;
}