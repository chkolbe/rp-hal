use core::cell::RefCell;
use core::convert::Infallible;
use defmt::*;

use hal::onewire::{Command, OneMaster, RomId};

use hal::blocking::delay::{DelayMs, DelayUs};
use hal::digital::v2::{InputPin, OutputPin};

use crate::gpio::{Floating, Input, Pin, PinId};

pub struct OneWire<'a, D: DelayUs<u32> + DelayMs<u32>, I: PinId> {
    // Safety: Option is not a Problem all Functions require &mut self or self.
    pin: Option<Pin<I, Input<Floating>>>,
    delay: &'a RefCell<D>,
}

impl<'a, D: DelayUs<u32> + DelayMs<u32>, I: PinId> OneWire<'a, D, I> {
    pub fn new(delay: &'a RefCell<D>, pin: Pin<I, Input<Floating>>) -> Self {
        OneWire {
            pin: Some(pin),
            delay,
        }
    }
}

impl<D: DelayUs<u32> + DelayMs<u32>, I: PinId> OneMaster for OneWire<'_, D, I> {
    type Error = Infallible;

    /// Does Bus Reset and syncs the Slaves
    ///
    /// **NOTE** Bus Reset should be done before any Slave Interaction.
    fn bus_reset(&mut self) -> Result<(), nb::Error<Infallible>> {
        let mut ow_pin = self.pin.take().unwrap().into_push_pull_output();
        // Use Pin as Output to drive Voltage down.
        ow_pin.set_low().unwrap();
        // Wait Sensor Reset Time 480us
        self.delay.borrow_mut().delay_us(480);
        // Use Pin as Input with Pull-Up to pull Voltage up.
        ow_pin.set_high().unwrap();
        let ow_pin = ow_pin.into_floating_input();

        let bus_sensor_present: bool;
        // Sample Sensor Presence Detect
        self.delay.borrow_mut().delay_us(90);
        // Check Sensor Presence True = yes, False = no
        bus_sensor_present = ow_pin.is_low().unwrap();
        // Wait Reset High time
        self.delay.borrow_mut().delay_us(390);

        self.pin = Option::Some(ow_pin);
        info!("Sensor Present: {}", bus_sensor_present);
        Ok(())
    }

    /// Write the OneWire Command on the Bus.
    fn write(
        &mut self,
        _rom_id: RomId,
        command: Command,
    ) -> Result<(), nb::Error<Infallible>> {
        let byte: u8 = command.into();

        for uc in 0..8 {
            let time_drive_bus_low: u32;
            let time_slot: u32;

            if (byte & (1 << uc)) != 0 {
                // Master Write Bit - 1
                time_drive_bus_low = 7;
                time_slot = 60 + 1 - time_drive_bus_low;
            } else {
                // Master Write Bit - 0
                time_drive_bus_low = 60;
                time_slot = 60 + 1 - time_drive_bus_low;
            }

            // Drive Bus to Low
            let mut ow_pin = self.pin.take().unwrap().into_push_pull_output();

            ow_pin.set_low().unwrap();
            // Wait average tLOW
            self.delay.borrow_mut().delay_us(time_drive_bus_low);
            // Drive Bus to High
            let ow_pin = ow_pin.into_floating_input();
            // Wait rest of tSLOT 60us - tLOW
            self.delay.borrow_mut().delay_us(time_slot);

            let ow_pin = ow_pin.into_floating_input();
            self.pin = Option::Some(ow_pin);
        }

        Ok(())
    }

    /// Read the Payload from the Bus.
    ///
    /// **NOTE** A Slave must be select and command must been sent before the Slave response.
    fn read(
        &mut self,
        _rom_id: RomId,
        buffer: &mut [u8],
    ) -> Result<(), nb::Error<Infallible>> {
        buffer.iter_mut().for_each(|byte| {
            *byte = 0;
            (0..8).for_each(|bit_position| {
                // Drive Bus to Low (Signal Master Read)
                let mut ow_pin = self.pin.take().unwrap().into_push_pull_output();
                ow_pin.set_low().unwrap();
                // wait tINT 2us
                self.delay.borrow_mut().delay_us(2);
                // Drive Bus to High (Wait for Sensor)
                let ow_pin = ow_pin.into_floating_input();
                // Wait for Sampling at 11us (2us + 9us)
                self.delay.borrow_mut().delay_us(9);
                // Sample Sensor Data Bit
                if ow_pin.is_high().unwrap() == true {
                    *byte |= 1 << bit_position;
                }

                // Wait rest of Slot 60us + 1us Bit-Spacing
                self.delay.borrow_mut().delay_us(60 + 1 - 9 - 2);

                self.pin = Option::Some(ow_pin);
            });
        });

        info!("{}", buffer);
        Ok(())
    }
}