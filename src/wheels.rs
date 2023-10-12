use arduino_hal::{
    port::{mode::PwmOutput, Pin},
    simple_pwm::PwmPinOps,
};
use micromath::F32;

use crate::println;

pub(crate) struct WheelTank<
    PLF: PwmPinOps<T>,
    PRF: PwmPinOps<T>,
    PLB: PwmPinOps<T>,
    PRB: PwmPinOps<T>,
    T,
> {
    leftf: Pin<PwmOutput<T>, PLF>,
    rightf: Pin<PwmOutput<T>, PRF>,
    leftb: Pin<PwmOutput<T>, PLB>,
    rightb: Pin<PwmOutput<T>, PRB>,
    leftv: f32,
    rightv: f32,
}

impl<T, PLF: PwmPinOps<T>, PRF: PwmPinOps<T>, PLB: PwmPinOps<T>, PRB: PwmPinOps<T>>
    WheelTank<PLF, PRF, PLB, PRB, T>
{
    /// Sets the power of the left side from -1.0 to 1.0
    pub(crate) fn set_left(&mut self, power: f32) {
        if power < 0.0 {
            self.leftb.set_duty((-power * 255.) as u8);
        } else {
            self.leftf.set_duty((-power * 255.) as u8);
        }
    }
    /// Sets the power of the right side from -1.0 to 1.0
    pub(crate) fn set_right(&mut self, power: f32) {
        if power < 0.0 {
            self.rightb.set_duty((-power * 255.) as u8);
        } else {
            self.rightf.set_duty((-power * 255.) as u8);
        }
    }

    /// power: From -1.0 going fully backward to 1.0 fully forward
    /// turn: From -1.0 going fully left to 1.0 fully right
    pub(crate) fn turn(&mut self, power: f32, turn: f32) {
        if power > 1.0 || power < -1.0 || turn < -1.0 || turn > 1.0 {
            println!(
                "Power or turn value not available p:{} t:{}",
                (power * 1000.) as u32,
                (turn * 1000.) as u32
            );
            return;
        }
        let (main, var) = if turn < 0.0 {
            (&mut self.rightv, &mut self.leftv)
        } else {
            (&mut self.leftv, &mut self.rightv)
        };
        let tv = 1.0 - F32(turn).abs().0 * 2.0;
        *main = power;
        *var = power * tv;
        self.set_left(self.leftv);
        self.set_right(self.rightv);
    }
}
