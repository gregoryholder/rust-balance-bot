#![allow(dead_code)]

use core::cell::RefCell;

use arduino_hal::delay_us;
use avr_device::interrupt::{CriticalSection, Mutex};
use embedded_hal::digital::OutputPin;

use crate::println;

pub struct Stepper<E, D, S> {
    enable_pin: E,
    direction_pin: D,
    step_pin: S,

    /// in ticks/second/second
    target_acceleration: u32,

    /// in milliticks/second
    target_speed: i32,

    /// in milliticks/second
    current_speed: i32,

    enabled: bool,

    /// measured in millisteps (10^-3 steps)
    current_millistep: i32,

    /// measured in millisteps (10^-3 steps)
    target_millistep: i32,

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
    fn run(&mut self, now: u64);
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
        self.target_speed = speed * 1_000;
    }

    fn run(&mut self, now: u64) {
        if !self.enabled {
            return;
        }

        let Some(last_step_time_us) = self.last_step_time_us else {
            self.last_step_time_us = Some(now);
            return;
        };

        let delta_us = (now - last_step_time_us) as u32;
        self.last_step_time_us = Some(now);

        let current_speed = self.update_speed(now);

        let target_millistep_delta = current_speed * delta_us as i32 / 1_000_000;

        // limit the target millistep delta to (slightly less than) a single step
        // this function can at most only move one full step, so anything more than that
        // is a sign that the target speed is too high
        let target_millistep_delta = target_millistep_delta.max(-999).min(999);

        let next_target_millistep = self.target_millistep + target_millistep_delta;

        self.target_millistep = next_target_millistep;

        let millistep_delta = next_target_millistep - self.current_millistep;

        if millistep_delta.abs() < 1_000 {
            return;
        }

        let direction = if millistep_delta > 0 {
            Direction::Forward
        } else {
            Direction::Backward
        };

        self.step(direction);

        return;
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

            max_speed: 2000,

            enabled: false,

            current_millistep: 0,
            target_millistep: 0,

            current_speed: 0,
            last_step_time_us: None,
            last_speed_update_us: None,
        }
    }

    /// performs a single step in the given direction and updates the current millistep
    fn step(&mut self, direction: Direction) {
        self.set_direction(direction.into());

        self.step_pin.set_high().unwrap();
        delay_us(1);
        self.step_pin.set_low().unwrap();

        // update current millistep based on the direction
        self.current_millistep += match direction {
            Direction::Forward => 1_000,
            Direction::Backward => -1_000,
        };
    }

    /// updates the current speed based on the target speed and acceleration and returns the new current speed
    fn update_speed(&mut self, now: u64) -> i32 {
        let current_speed = self.current_speed;

        let Some(last_speed_update_us) = self.last_speed_update_us else {
            self.last_speed_update_us = Some(now);
            return current_speed;
        };

        let speed_update_delta_us = (now - last_speed_update_us) as u32;

        if speed_update_delta_us < 5000 {
            return current_speed;
        }

        self.last_speed_update_us = Some(now);

        let new_speed = calculate_new_speed(
            current_speed,
            self.target_speed,
            self.target_acceleration,
            speed_update_delta_us,
        );

        self.current_speed = new_speed;

        // print target speed and current speed
        // println!("target speed: {}, current speed: {}", self.target_speed, new_speed);

        // print current millistep and target millistep
        // println!("current millistep: {}, target millistep: {}", self.current_millistep, self.target_millistep);

        new_speed
    }

    fn set_direction(&mut self, direction: bool) {
        self.direction_pin.set_state(direction.into()).unwrap();
    }
}

fn calculate_new_speed(
    current_speed: i32,
    target_speed: i32,
    target_acceleration: u32,
    speed_update_delta_us: u32,
) -> i32 {
    let speed_delta = target_acceleration * speed_update_delta_us / 1_000;

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
    fn run(&self, now: u64);
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

    fn run(&self, now: u64) {
        avr_device::interrupt::free(|cs| {
            if let Some(stepper) = self.0.borrow(cs).borrow_mut().as_mut() {
                stepper.run(now);
            }
        });
    }
}
