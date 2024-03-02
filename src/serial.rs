use arrayvec::ArrayString;
use avr_device::interrupt::Mutex;
use core::cell::RefCell;

pub type Usart = arduino_hal::hal::usart::Usart0<arduino_hal::DefaultClock>;
pub static GLOBAL_SERIAL: Mutex<RefCell<Option<&mut Usart>>> = Mutex::new(RefCell::new(None));

#[allow(unused)]
pub fn init(serial: &'static mut Usart) {
    avr_device::interrupt::free(|cs| {
        GLOBAL_SERIAL.borrow(cs).replace(Some(serial));
    })
}

#[macro_export]
macro_rules! println {
    ($($arg:tt)*) => {
        use ::arduino_hal::prelude::*;
        ::avr_device::interrupt::free(|cs| {
            if let Some(serial) = &mut *crate::serial::GLOBAL_SERIAL.borrow(cs).borrow_mut() {
                ::ufmt::uwriteln!(serial, $($arg)*).unwrap_infallible()
            }
        })
    }
}

#[macro_export]
macro_rules! print {
    ($($arg:tt)*) => {
        use ::arduino_hal::prelude::*;
        ::avr_device::interrupt::free(|cs| {
            if let Some(serial) = &mut *crate::serial::GLOBAL_SERIAL.borrow(cs).borrow_mut() {
                ::ufmt::uwrite!(serial, $($arg)*).unwrap_infallible()
            }
        })
    }
}

use arduino_hal::prelude::*;

pub trait ReadLine {
    fn try_read_line<const L: usize>(&mut self) -> Option<ArrayString<L>>;
}

impl ReadLine for Usart {
    fn try_read_line<const L: usize>(&mut self) -> Option<ArrayString<L>> {
        let mut line = ArrayString::<L>::new();

        let Ok(b) = self.read() else {
            return None;
        };

        let mut c = b as char;

        loop {
            if c == '\n' {
                return Some(line);
            }
            if line.try_push(c).is_err() {
                return Some(line);
            }
            c = self.read_byte() as char;
        }
    }
}
