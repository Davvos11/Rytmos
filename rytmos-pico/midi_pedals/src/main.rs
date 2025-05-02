#![no_std]
#![no_main]

#[link_section = ".boot2"]
#[no_mangle]
#[used]
pub static BOOT2_FIRMWARE: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

use common::debouncer::Debouncer;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;
use heapless::Vec;
use itertools::izip;
use panic_probe as _;
use rp_pico::hal::timer::Instant;
use rp_pico::hal::Timer;
use rp_pico::{
    entry,
    hal::{
        clocks::{init_clocks_and_plls, Clock},
        gpio::{self, FunctionSio, Pin, PullUp, SioInput},
        sio::Sio,
        usb::UsbBus,
        Watchdog,
    },
    pac,
};
use usb_device::device::StringDescriptors;
use usb_device::{
    bus::UsbBusAllocator,
    device::{UsbDeviceBuilder, UsbVidPid},
};
use usbd_midi::message::{ControlFunction, U7};
use usbd_midi::CableNumber;
use usbd_midi::Message::ControlChange;
use usbd_midi::{message::Channel, UsbMidiClass};

const PEDALS: usize = 4;
const CC_BANK_A: u8 = 1;
const CC_BANK_B: u8 = 64;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let sio = Sio::new(pac.SIO);

    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let clocks = init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let pins = gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    info!("Creating usb devices.");

    let usb_bus = UsbBusAllocator::new(UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    let mut led_pin = pins.gpio25.into_push_pull_output();

    let pedal_pins: [Pin<_, FunctionSio<SioInput>, PullUp>; PEDALS] = [
        pins.gpio0.reconfigure().into_dyn_pin(),
        pins.gpio1.reconfigure().into_dyn_pin(),
        pins.gpio2.reconfigure().into_dyn_pin(),
        pins.gpio3.reconfigure().into_dyn_pin(),
    ];
    let mut pedal_debouncers: Vec<_, PEDALS> =
        pedal_pins.iter().map(|_| Debouncer::new(1000)).collect();

    let switch1: Pin<_, FunctionSio<SioInput>, PullUp> = pins.gpio12.reconfigure();
    let switch2: Pin<_, FunctionSio<SioInput>, PullUp> = pins.gpio13.reconfigure();
    let mut switch1_deb = Debouncer::new(1000);
    let mut switch2_deb = Debouncer::new(1000);

    let control_changes_a = create_ccs(CC_BANK_A);
    let control_changes_b = create_ccs(CC_BANK_B);

    led_pin.set_high().unwrap();

    let mut midi = UsbMidiClass::new(&usb_bus, 1, 0).unwrap();

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0xabc9, 0xee0))
        .device_class(0)
        .device_sub_class(0)
        .strings(&[StringDescriptors::default()
            .manufacturer("de vck")
            .product("midi pedals lmao")
            .serial_number("0")])
        .unwrap()
        .build();

    info!("usb device is created");

    let mut control_changes = if switch1.is_high().unwrap() {
        info!(
            "Starting using bank A (switch 1: {})",
            switch1.is_high().unwrap()
        );
        &control_changes_a
    } else {
        info!(
            "Starting using bank B (switch 1: {})",
            switch1.is_high().unwrap()
        );
        &control_changes_b
    };
    let mut double_cancellation = switch2.is_high().unwrap();
    info!(
        "Starting with double-press cancellation = {}",
        double_cancellation
    );
    // Create counters for each pedal (0: last time pedal up, 1: last time pedal down)
    let mut last_presses: Vec<_, PEDALS> =
        pedal_pins.iter().map(|_| (Instant::from_ticks(0), Instant::from_ticks(0))).collect();
    let mut pressed = pedal_pins.iter().filter(|p| p.is_high().unwrap()).count();

    loop {
        // Poll USB device to keep connection alive
        usb_dev.poll(&mut [&mut midi]);

        switch1_deb.update(switch1.is_high().unwrap());
        switch2_deb.update(switch2.is_high().unwrap());

        // Determine the CC values based on the switch setting
        if switch1_deb.stable_rising_edge() {
            debug!("Switching to bank A");
            // Send pedal off to all B channels (when switching)
            for cc in &control_changes_b {
                send_midi_cc(&mut midi, cc, U7::MIN).ok();
                delay.delay_ms(1);
            }
            // Set current channels to A
            control_changes = &control_changes_a;
        } else if switch1_deb.stable_falling_edge() {
            debug!("Switching to bank B");
            for cc in &control_changes_a {
                send_midi_cc(&mut midi, cc, U7::MIN).ok();
                delay.delay_ms(1);
            }
            // Set current channels to B
            control_changes = &control_changes_b;
        }

        // Determine if we double-press-cancellation is enabled
        if switch2_deb.stable_rising_edge() {
            debug!("Enabling double-press cancellation");
            double_cancellation = true;
        } else if switch2_deb.stable_falling_edge() {
            debug!("Disabling double-press cancellation");
            double_cancellation = false;
        }

        for (pin, debouncer, cc, last_pressed) in izip!(
            &pedal_pins,
            &mut pedal_debouncers,
            control_changes,
            &mut last_presses
        ) {
            debouncer.update(pin.is_high().unwrap());
            let mut up = false;
            let mut down = false;

            if debouncer.stable_falling_edge() {
                down = true;
                debug!("{} down (MIDI {})", pin.id().num, u8::from(cc.0.clone()));
                pressed += 1;
            }

            if debouncer.stable_rising_edge() {
                up = true;
                debug!("{} up (MIDI {})", pin.id().num, u8::from(cc.0.clone()));
                pressed = pressed.saturating_sub(1);
            }
            
            if up || down {
                // Determine time since last up or down event and determine if we should cancel
                let elapsed = if up {
                    timer.get_counter().checked_duration_since(last_pressed.0)
                } else {
                    timer.get_counter().checked_duration_since(last_pressed.1)
                };
                let should_cancel = elapsed.map(|e| e.to_millis() <= 500).unwrap_or(false);
                if double_cancellation && should_cancel {
                    debug!("Cancelled sending midi ({:?} ms since last press)", elapsed.map(|e| e.to_millis()));
                }
                // Send MIDI signal (or cancel)
                if !double_cancellation || !should_cancel {
                    let value = if up {U7::MIN} else {U7::MAX};
                    send_midi_cc(&mut midi, cc, value).ok();
                }
                // Update timers
                if up {
                    last_pressed.0 = timer.get_counter();
                } else {
                    last_pressed.1 = timer.get_counter();
                }
            }
        }

        if pressed > 0 {
            led_pin.set_low().ok();
        } else {
            led_pin.set_high().ok();
        }
    }
}

fn send_midi_cc(
    midi: &mut UsbMidiClass<UsbBus>,
    cc: &ControlFunction,
    value: U7,
) -> usb_device::Result<usize> {
    midi.send_packet(
        ControlChange(Channel::Channel1, cc.clone(), value).into_packet(CableNumber::Cable0),
    )
}

fn create_ccs(offset: u8) -> Vec<ControlFunction, PEDALS> {
    (0..PEDALS as u8)
        .map(|i| ControlFunction((i + offset).try_into().unwrap()))
        .collect()
}
