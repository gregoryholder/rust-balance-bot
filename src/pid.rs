// allow unused code
#![allow(dead_code, unused_variables)]

#[derive(Default)]
pub struct PID {
    k_p: i32,
    k_i: i32,
    k_d: i32,

    error: i32,

    /// scaled by 1000
    integral: i32,
    integral_max: u32,

    // keep track of derivative for debugging
    derivative_prior_filtered: i32,
    derivative_prior_raw: i32,

    output: i32,
}

impl PID {
    const ALPHA: i32 = 30;
    pub fn new() -> Self {
        Self {
            integral_max: 300,
            ..Self::default()
        }
    }

    pub fn new_with_constants(k_p: i32, k_i: i32, k_d: i32, integral_max: u32) -> Self {
        Self {
            k_p,
            k_i,
            k_d,
            integral_max,
            ..Self::default()
        }
    }

    pub fn update(&mut self, error: i32, delta_ms: u32) {
        let integral_max = self.integral_max as i32;
        let delta_ms = delta_ms as i32;

        let integral = self.integral + error * delta_ms;
    
        self.integral = integral.clamp(-integral_max * 1000, integral_max * 1000);

        let derivative_raw = (error - self.error) * 10 / delta_ms;

        let derivative_filtered = (self.derivative_prior_filtered * Self::ALPHA
            + derivative_raw
            + self.derivative_prior_raw)
            / (Self::ALPHA + 2);

        self.derivative_prior_filtered = derivative_filtered;
        self.derivative_prior_raw = derivative_raw;

        self.error = error;

        let p: i32 = self.k_p as i32 * self.error as i32 * 10;
        let i: i32 = self.k_i * self.integral /10;
        let d: i32 = self.k_d * self.derivative_prior_filtered * 100;

        self.output = p + i + d;
    }

    pub fn get_output(&self) -> i32 {
        self.output
    }

    pub fn set_p(&mut self, p: i32) {
        self.k_p = p;
    }

    pub fn set_i(&mut self, i: i32) {
        // scale integral to prevent a sudden jump in output
        if self.k_i != 0 {
            self.integral = self.integral * i / self.k_i;
        } else {
            self.integral = 0;
        }

        self.k_i = i;
    }

    pub fn set_d(&mut self, d: i32) {
        self.k_d = d;
    }

    pub fn get_p(&self) -> i32 {
        self.k_p
    }

    pub fn get_i(&self) -> i32 {
        self.k_i
    }

    pub fn get_d(&self) -> i32 {
        self.k_d
    }

    pub fn get_integral(&self) -> i32 {
        self.integral
    }

    pub fn get_derivative(&self) -> i32 {
        self.derivative_prior_filtered
    }

    pub fn clear_integral(&mut self) {
        self.integral = 0;
    }
}
