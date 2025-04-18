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
use usbd_midi::{
    message::{Channel},
    UsbMidiClass,
};

const PEDALS: usize = 2;

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

    let _delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

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
    ];

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

    let mut debouncers: Vec<_, PEDALS> = pedal_pins.iter().map(|_| Debouncer::new(1000)).collect();

    let ccs: Vec<_, PEDALS> = pedal_pins
        .iter()
        .enumerate()
        .map(|(i, _)| ControlFunction((i as u8 + 1).try_into().unwrap()))
        .collect();

    led_pin.set_high().unwrap();

    loop {
        usb_dev.poll(&mut [&mut midi]);

        for (pin, debouncer, cc) in izip!(&pedal_pins, &mut debouncers, &ccs) {
            debouncer.update(pin.is_high().unwrap());

            if debouncer.stable_falling_edge() {
                midi.send_packet(
                    ControlChange(Channel::Channel1, cc.clone(), U7::MAX)
                        .into_packet(CableNumber::Cable0),
                )
                .ok();
            }

            if debouncer.stable_rising_edge() {
                midi.send_packet(
                    ControlChange(Channel::Channel1, cc.clone(), U7::MIN)
                        .into_packet(CableNumber::Cable0),
                )
                .ok();
            }
        }
    }
}
