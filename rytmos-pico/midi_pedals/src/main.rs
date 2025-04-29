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
        pins.gpio16.reconfigure().into_dyn_pin(),
        pins.gpio17.reconfigure().into_dyn_pin(),
        pins.gpio18.reconfigure().into_dyn_pin(),
        pins.gpio19.reconfigure().into_dyn_pin(),
    ];
    let mut pedal_debouncers: Vec<_, PEDALS> =
        pedal_pins.iter().map(|_| Debouncer::new(1000)).collect();

    let switch_pin: Pin<_, FunctionSio<SioInput>, PullUp> = pins.gpio14.reconfigure();
    let mut switch_debouncer = Debouncer::new(1000);

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

    let mut control_changes = &control_changes_a;
    let mut pressed = pedal_pins.iter().filter(|p| p.is_high().unwrap()).count();

    loop {
        // Poll USB device to keep connection alive
        usb_dev.poll(&mut [&mut midi]);

        // Determine the CC values based on the switch setting
        switch_debouncer.update(switch_pin.is_high().unwrap());
        if switch_debouncer.stable_rising_edge() {
            // Send pedal off to all B channels (when switching)
            for cc in &control_changes_b {
                send_midi_cc(&mut midi, cc, U7::MIN).ok();
                delay.delay_ms(1);
            }
            // Set current channels to A
            control_changes = &control_changes_a;
        } else if switch_debouncer.stable_falling_edge() {
            for cc in &control_changes_a {
                send_midi_cc(&mut midi, cc, U7::MIN).ok();
                delay.delay_ms(1);
            }
            // Set current channels to B
            control_changes = &control_changes_b;
        }

        for (pin, debouncer, cc) in izip!(&pedal_pins, &mut pedal_debouncers, control_changes) {
            debouncer.update(pin.is_high().unwrap());

            if debouncer.stable_falling_edge() {
                send_midi_cc(&mut midi, cc, U7::MAX).ok();
                pressed += 1;
            }

            if debouncer.stable_rising_edge() {
                send_midi_cc(&mut midi, cc, U7::MIN).ok();
                pressed -= 1;
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
