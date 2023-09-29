#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

use arduino_hal::{
    port::{mode::PwmOutput, Pin},
    simple_pwm::{IntoPwmPin, PwmPinOps, Timer1Pwm},
};
use panic_halt as _;

//Taken from https://github.com/Rahix/avr-hal/blob/main/examples/arduino-uno/src/bin/uno-millis.rs
const PRESCALER: u64 = 1024;
const TIMER_COUNTS: u64 = 125;

const MILLIS_INCREMENT: u64 = PRESCALER * TIMER_COUNTS / 16000;

static MILLIS_COUNTER: avr_device::interrupt::Mutex<core::cell::Cell<u64>> =
    avr_device::interrupt::Mutex::new(core::cell::Cell::new(0));

fn millis_init(tc0: arduino_hal::pac::TC0) {
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

    // Reset the global millisecond counter
    avr_device::interrupt::free(|cs| {
        MILLIS_COUNTER.borrow(cs).set(0);
    });
}

#[avr_device::interrupt(atmega328p)]
fn TIMER0_COMPA() {
    avr_device::interrupt::free(|cs| {
        let counter_cell = MILLIS_COUNTER.borrow(cs);
        let counter = counter_cell.get();
        counter_cell.set(counter + MILLIS_INCREMENT);
    })
}

fn millis() -> u64 {
    avr_device::interrupt::free(|cs| MILLIS_COUNTER.borrow(cs).get())
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
    let mut timer = Timer1Pwm::new(dp.TC1, arduino_hal::simple_pwm::Prescaler::Prescale64);

    let mut led = pins.d13.into_output().into_pwm(&mut timer);
    let mut led2 = pins.d12.into_output().into_pwm(&mut timer);
    // let mut dir = true;
    // let mut strength : i16 = 0;
    // let mut buffer: [usize;4096] = [0; 4096];
    millis_init(dp.TC0);
    let mut bounds = (0, 255);
    let accel = |x: f32| x * x;
    let mut ease = MotorEase::new(bounds.0, bounds.1, 1000);
    let mut ease2 = MotorEase::new_with_interp(bounds.0, bounds.1, 1000, &accel);

    // Enable interrupts globally
    unsafe { avr_device::interrupt::enable() };
    loop {
        // led.set_duty(strength as u8);
        // strength+= if dir {1} else {-1};
        // if strength == 255 || strength == 0 {
        //     dir = !dir;
        // }
        // buffer[strength as usize] += 1;
        arduino_hal::delay_ms(1000 / 30);
        do_ease(&mut ease, &mut led);
        do_ease(&mut ease2, &mut led2);
        if matches!(ease, MotorEase::Done) {
            (bounds.1, bounds.0) = (bounds.0, bounds.1);
            ease = MotorEase::new(bounds.0, bounds.1, 1000);
        }
        if matches!(ease2, MotorEase::Done) {
            ease2 = MotorEase::new_with_interp(bounds.0, bounds.1, 1000, &accel);
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

        let strength = *start + ((*end - *start) as f32 * interp(prog)) as u8;
        p.set_duty(strength);
        if cur > *start_ms + *dur {
            *me = MotorEase::Done;
        }
    }
}
