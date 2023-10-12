#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

mod wheels;

use core::cell::RefCell;

use arduino_hal::port::mode::Output;
use arduino_hal::port::PinOps;
use arduino_hal::simple_pwm::Timer1Pwm;
use arduino_hal::{
    port::{mode::PwmOutput, Pin},
    simple_pwm::{IntoPwmPin, PwmPinOps},
};
use avr_device::interrupt::Mutex;
use micromath::F32;
// use num_traits::Float;
use panic_halt as _;
use ufmt::uDebug;
// use ufmt::derive::uDebug;

#[macro_export]
macro_rules! print {
    ($($t:tt)*) => {
        interrupt::free(
            |cs| {
                if let Some(console) = crate::CONSOLE.borrow(cs).borrow_mut().as_mut() {
                    let _ = ufmt::uwrite!(console, $($t)*);
                }
            },
        )
    };
}

#[macro_export]
macro_rules! println {
    ($($t:tt)*) => {
        avr_device::interrupt::free(
            |cs| {
                if let Some(console) = crate::CONSOLE.borrow(cs).borrow_mut().as_mut() {
                    let _ = ufmt::uwriteln!(console, $($t)*);
                }
            },
        )
    };
}

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    /*
     * For examples (and inspiration), head to
     *
     *     https://github.com/Rahix/avr-hal/tree/main/examples
     *
     * NOTE: Not all examples were ported to all boards!  There is a good chance though, that code
     * for a different board can be adapted for yours.  The Arduino Uno currently has the most
     * examples available.
     */
    #[cfg(feature = "mega")]
    let mut timer = Timer1Pwm::new(dp.TC1, arduino_hal::simple_pwm::Prescaler::Prescale64);
    #[cfg(feature = "uno")]
    let timer = Timer1Pwm::new(dp.TC1, arduino_hal::simple_pwm::Prescaler::Prescale64);

    #[cfg(feature = "mega")]
    let mut led = pins.d13.into_output().into_pwm(&mut timer);
    #[cfg(feature = "uno")]
    let mut led = pins.d10.into_output().into_pwm(&timer);

    #[cfg(feature = "mega")]
    let mut led2 = pins.d12.into_output().into_pwm(&mut timer);
    #[cfg(feature = "uno")]
    let mut led2 = pins.d9.into_output().into_pwm(&timer);

    #[cfg(feature = "uno")]
    let mut ledt = pins.d13.into_output();
    #[cfg(feature = "mega")]
    let mut ledt = pins.d11.into_output();
    // let mut dir = true;
    // let mut strength : i16 = 0;
    // let mut buffer: [usize;4096] = [0; 4096];
    timer_init(dp.TC0);
    // let val = 0;
    let mut bounds = (0, 255);
    let mid_ease = |x: f32| {
        // let x2 = F32(2.0 * x - 1.0).powi(3).0;
        // 0.5 * x2 + 0.5 - 0.3 * x

        // let v = F32(if x < 0.5 { x } else { 1.0 - x } * 3.141592 * 3.0);
        // v.sin().0
        (F32((x - 0.5) * 3.141592).sin().0 + 1.0) / 2.0
    };
    let dur = 2000;
    let mut ease = MotorEase::new(bounds.0, bounds.1, dur);
    let mut ease2 = MotorEase::new_with_interp(bounds.0, bounds.1, dur, &mid_ease);

    let serial = arduino_hal::default_serial!(dp, pins, 57600);
    avr_device::interrupt::free(|cs| *CONSOLE.borrow(cs).borrow_mut() = Some(serial));
    led.enable();
    led2.enable();
    // let sq = Square::new(440., 0.);
    let sound = false;
    let tones: &'static [u8] = &[100, 150, 255, 125, 255, 200, 125];
    let mut tc = 0;
    let tdir = 200;
    if sound {
        ease = MotorEase::new(tones[0], tones[0], tdir);
    }
    // let analog = arduino_hal::i2c::I2c::new(I2C, ledt, ledt, 10000);

    // Enable interrupts globally
    unsafe { avr_device::interrupt::enable() };
    loop {
        // led.set_duty(strength as u8);
        // strength+= if dir {1} else {-1};
        // if strength == 255 || strength == 0 {
        //     dir = !dir;
        // }
        // buffer[strength as usize] += 1;

        // #[cfg(feature = "uno")]
        // ledt.toggle();
        if sound {
            // sq.update(&mut ledt);
            do_ease(&mut ease, &mut led);
            if matches!(ease, MotorEase::Done) {
                tc = (tc + 1) % tones.len();
                ease = MotorEase::new(tones[tc], tones[tc], tdir);
            }
        } else {
            arduino_hal::delay_ms(1000 / 60);
            do_ease(&mut ease, &mut led);
            do_ease(&mut ease2, &mut led2);
            // let ctrl = led2.get_duty();
            // serial.write_byte(ctrl);
            // ufmt::uwriteln!(serial, "{}", ctrl).unwrap();
            // println!("{} ", ctrl);
            if matches!(ease, MotorEase::Done) {
                (bounds.1, bounds.0) = (bounds.0, bounds.1);
                ease = MotorEase::new(bounds.0, bounds.1, dur);
            }
            if matches!(ease2, MotorEase::Done) {
                ease2 = MotorEase::new_with_interp(bounds.0, bounds.1, dur, &mid_ease);
            }
        }
    }
}

enum MotorEase<'a> {
    Running {
        start: u8,
        end: u8,
        start_ms: u64,
        dur: u64,
        interp: &'a dyn Fn(f32) -> f32,
    },
    Done,
}

impl<'a> uDebug for MotorEase<'a> {
    fn fmt<W>(&self, f: &mut ufmt::Formatter<'_, W>) -> Result<(), W::Error>
    where
        W: ufmt::uWrite + ?Sized,
    {
        match self {
            MotorEase::Running {
                start,
                end,
                start_ms,
                dur,
                interp: _,
            } => ufmt::uwrite!(
                f,
                "MotorEase::Running{{start:{},end:{},start_ms:{},dur:{} }}",
                start,
                end,
                start_ms,
                dur
            ),
            MotorEase::Done => ufmt::uwrite!(f, "MororEase::Done"),
        }
    }
}

fn lin(x: f32) -> f32 {
    x
}

impl MotorEase<'static> {
    fn new(start: u8, end: u8, dur: u64) -> Self {
        let st = millis();
        Self::Running {
            start,
            end,
            start_ms: st,
            dur,
            interp: &(lin as fn(f32) -> f32),
        }
    }
}

impl<'a> MotorEase<'a> {
    // <F: Fn(f32) -> f32>
    fn new_with_interp<F: Fn(f32) -> f32>(start: u8, end: u8, dur: u64, interp: &'a F) -> Self {
        let st = millis();
        Self::Running {
            start,
            end,
            start_ms: st,
            dur,
            interp,
        }
    }
}

fn do_ease<P: PwmPinOps<T>, T>(me: &mut MotorEase, p: &mut Pin<PwmOutput<T>, P>) {
    if let MotorEase::Running {
        start,
        end,
        start_ms,
        dur,
        interp,
    } = me
    {
        // let interp = &lin;
        let cur = millis();
        let prog = ((cur - *start_ms) as f32) / (*dur as f32);

        let strength = (*start as f32) + ((*end as f32 - *start as f32) * interp(prog));
        // println!("s: {}", (strength * 128.) as u64);
        p.set_duty(strength as u8);

        if cur > *start_ms + *dur {
            *me = MotorEase::Done;
        }
    }
}

struct Square {
    wl: f32,
    phase: f32,
}

impl Square {
    fn new(freq: f32, phase: f32) -> Self {
        Self {
            wl: 1_000_000. / freq,
            phase,
        }
    }

    fn update<P: PinOps>(&self, pin: &mut Pin<Output, P>) {
        let ms = micros();

        let rem = (ms as f32) - F32((ms as f32 - self.phase) / self.wl).floor().0 * self.wl;
        let p = rem / self.wl;
        println!("{}", (p * 1000.) as u16);
        if p > 0.5 {
            pin.set_high();
        } else {
            pin.set_low();
        }
    }
}

// struct Morse<'a> {
//     data: &'a str,
//     progress: usize,
//     morse: Option<MorseLetter>,
// }

// struct MorseLetterCode {
//     Single([bool; 1]),
//     Double([bool; 2]),
//     Triple([bool; 3]),
//     Four([bool; 4]),
//     Five([bool; 5]),
// }

// struct MorseLetter {
//     m: MorseLetterCode,
//     prog: f32,
// }

// impl TryFrom<u8> for MorseLetterCode {
//     type Error = ();

//     fn try_from(value: u8) -> Result<Self, Self::Error> {
//         let c: char = value.into();
//         match c {
//             's' => Ok(Self::Single())
//             _ => Err(()),
//         }
//     }
// }

// impl<'a> Morse<'a> {
//     fn new(data: &'a str, time: f32) -> Self {
//         Self {
//             data,
//             progress: 0,
//             morse: None,
//         }
//     }

//     fn update<P: PinOps>(&self, pin: &mut Pin<Output, P>) {
//         let ms = micros();

//         let rem = (ms as f32) - F32((ms as f32 - self.phase) / self.wl).floor().0 * self.wl;
//         let p = rem / self.wl;
//         println!("{}", (p * 1000.) as u16);
//         if p > 0.5 {
//             pin.set_high();
//         } else {
//             pin.set_low();
//         }
//     }
// }

//Taken from https://github.com/Rahix/avr-hal/blob/main/examples/arduino-uno/src/bin/uno-millis.rs
const PRESCALER: u64 = 1024;
const TIMER_COUNTS: u64 = 1;

const MICROS_INCREMENT: u64 = PRESCALER * TIMER_COUNTS / 16;

static MICROS_COUNTER: avr_device::interrupt::Mutex<core::cell::Cell<u64>> =
    avr_device::interrupt::Mutex::new(core::cell::Cell::new(0));

fn timer_init(tc0: arduino_hal::pac::TC0) {
    // Configure the timer for the above interval (in CTC mode)
    // and enable its interrupt.
    tc0.tccr0a.write(|w| w.wgm0().ctc());
    tc0.ocr0a.write(|w| w.bits(TIMER_COUNTS as u8));
    tc0.tccr0b.write(|w| match PRESCALER {
        8 => w.cs0().prescale_8(),
        64 => w.cs0().prescale_64(),
        256 => w.cs0().prescale_256(),
        1024 => w.cs0().prescale_1024(),
        _ => panic!(),
    });
    tc0.timsk0.write(|w| w.ocie0a().set_bit());

    // Reset the global microsecond counter
    avr_device::interrupt::free(|cs| {
        MICROS_COUNTER.borrow(cs).set(0);
    });
}

#[avr_device::interrupt(atmega328p)]
fn TIMER0_COMPA() {
    avr_device::interrupt::free(|cs| {
        let counter_cell = MICROS_COUNTER.borrow(cs);
        let counter = counter_cell.get();
        counter_cell.set(counter + MICROS_INCREMENT);
    })
}

fn millis() -> u64 {
    micros() / 1000
}

fn micros() -> u64 {
    avr_device::interrupt::free(|cs| MICROS_COUNTER.borrow(cs).get())
}

type Console = arduino_hal::hal::usart::Usart0<arduino_hal::DefaultClock>;
static CONSOLE: Mutex<RefCell<Option<Console>>> = Mutex::new(RefCell::new(None));
