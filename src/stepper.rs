#![allow(dead_code)]

use core::cell::RefCell;

use arduino_hal::{
    delay_us,
    port::{mode::Output, Pin},
};
use avr_device::interrupt::Mutex;
use embedded_hal::digital::OutputPin;

pub struct Stepper<E, D, S> {
    enable_pin: E,
    direction_pin: D,
    step_pin: S,

    /// in ticks/second/second
    target_acceleration: u32,

    /// in milliticks/second
    target_speed: i32,

    /// in milliticks/second
    pub current_speed: i32,

    enabled: bool,

    /// measured in millisteps (10^-3 steps)
    current_millistep: i32,

    /// measured in millisteps (10^-3 steps)
    target_millistep: i32,

    pub delta_step: i32,

    last_step_time_us: Option<u64>,
    last_speed_update_us: Option<u64>,

    max_speed: i32,
}

#[derive(PartialEq, Clone, Copy)]
pub enum Direction {
    Forward,
    Backward,
}

// implement into bool for direction
impl From<Direction> for bool {
    fn from(direction: Direction) -> bool {
        match direction {
            Direction::Forward => true,
            Direction::Backward => false,
        }
    }
}

pub trait StepperControl {
    fn set_enable(&mut self, enable: bool);
    fn set_speed(&mut self, speed: i32);
    fn set_acceleration(&mut self, acc: u32);

    fn update(&mut self, delta_ms: u32);

    /// perform a single step if necessary
    fn do_step(&mut self);

    // fn run(&mut self, now: u64);
    fn get_step_count(&self) -> i32;
}

impl<E, D, S> StepperControl for Stepper<E, D, S>
where
    E: OutputPin,
    D: OutputPin,
    S: OutputPin,
{
    fn set_acceleration(&mut self, acc: u32) {
        self.target_acceleration = acc;
    }

    fn set_speed(&mut self, speed: i32) {
        let speed = speed.max(-self.max_speed).min(self.max_speed);
        self.target_speed = speed;
    }

    fn update(&mut self, delta_ms: u32) {
        if !self.enabled {
            return;
        }

        // calculate the new speed based on the target speed, acceleration, and the time since the last update
        let current_speed = self.update_speed(delta_ms);

        // how many millisteps the motor should move in the current time period
        self.target_millistep += current_speed * delta_ms as i32 / 1_000;

        let delta_step = (self.target_millistep - self.current_millistep) / 1000;

        self.delta_step += delta_step;
        self.current_millistep += delta_step * 1000;
    }

    #[inline(always)]
    fn do_step(&mut self) {
        if self.delta_step == 0 {
            return;
        }

        if self.delta_step > 0 {
            self.direction_pin.set_high().unwrap();
            self.step_pin.set_high().unwrap();
            delay_us(1);
            self.step_pin.set_low().unwrap();

            self.delta_step -= 1;
        } else {
            self.direction_pin.set_low().unwrap();
            self.step_pin.set_high().unwrap();
            delay_us(1);
            self.step_pin.set_low().unwrap();
            self.delta_step += 1;
        }
    }

    fn set_enable(&mut self, enable: bool) {
        self.enable_pin.set_state((!enable).into()).unwrap();
        self.enabled = enable;

        if !enable {
            self.current_speed = 0;
            self.last_step_time_us = None;
            self.last_speed_update_us = None;
        }
    }

    fn get_step_count(&self) -> i32 {
        self.current_millistep / 1000
    }
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
            target_acceleration: 0,
            target_speed: 0,

            max_speed: 2000 * 1000,

            enabled: false,

            current_millistep: 0,
            target_millistep: 0,
            delta_step: 0,

            current_speed: 0,
            last_step_time_us: None,
            last_speed_update_us: None,
        }
    }

    /// updates the current speed based on the target speed and acceleration and returns the new current speed
    fn update_speed(&mut self, delta_ms: u32) -> i32 {
        let current_speed = self.current_speed;

        let new_speed = calculate_new_speed(
            current_speed,
            self.target_speed,
            self.target_acceleration,
            delta_ms,
        );

        self.current_speed = new_speed;

        new_speed
    }
}

fn calculate_new_speed(
    current_speed: i32,
    target_speed: i32,
    target_acceleration: u32,
    delta_ms: u32,
) -> i32 {
    let speed_delta = target_acceleration * delta_ms;

    // how much the speed needs to change before matching the target (can be + or -)
    let speed_target_diff = target_speed - current_speed;

    // if the current speed is close enough to the target speed, apply the target speed directly
    let new_speed = if (speed_target_diff.abs() as u32) < speed_delta {
        target_speed
    } else {
        // update the speed (towards the target speed)
        current_speed + speed_delta as i32 * speed_target_diff.signum()
    };

    new_speed
}

pub trait RefStepperControl {
    fn set_enable(&self, enable: bool);
    fn set_speed(&self, speed: i32);
    fn set_acceleration(&self, acc: u32);
    fn get_step_count(&self) -> i32;
}

pub struct StepperMutexControl<T>(pub Mutex<RefCell<Option<T>>>);

// implement stepper control for global mutex of stepper
impl<S> StepperMutexControl<S> {
    pub const fn new_empty() -> Self {
        StepperMutexControl(Mutex::new(RefCell::new(None)))
    }

    pub fn assign(&self, stepper: S) {
        avr_device::interrupt::free(|cs| {
            self.0.borrow(cs).replace(Some(stepper));
        });
    }
}

impl<S> RefStepperControl for StepperMutexControl<S>
where
    S: StepperControl,
{
    fn set_enable(&self, enable: bool) {
        avr_device::interrupt::free(|cs| {
            if let Some(stepper) = self.0.borrow(cs).borrow_mut().as_mut() {
                stepper.set_enable(enable);
            }
        });
    }

    fn set_speed(&self, speed: i32) {
        avr_device::interrupt::free(|cs| {
            if let Some(stepper) = self.0.borrow(cs).borrow_mut().as_mut() {
                stepper.set_speed(speed);
            }
        });
    }

    fn set_acceleration(&self, acc: u32) {
        avr_device::interrupt::free(|cs| {
            if let Some(stepper) = self.0.borrow(cs).borrow_mut().as_mut() {
                stepper.set_acceleration(acc);
            }
        });
    }

    fn get_step_count(&self) -> i32 {
        avr_device::interrupt::free(|cs| {
            if let Some(stepper) = self.0.borrow(cs).borrow_mut().as_mut() {
                stepper.get_step_count()
            } else {
                0
            }
        })
    }
}
