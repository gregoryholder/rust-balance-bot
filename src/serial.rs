use avr_device::interrupt::Mutex;
use core::cell::RefCell;

pub type Usart = arduino_hal::hal::usart::Usart0<arduino_hal::DefaultClock>;
pub static GLOBAL_SERIAL: Mutex<RefCell<Option<Usart>>> = Mutex::new(RefCell::new(None));


#[allow(unused)]
pub fn init(serial: Usart) {
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
