#![no_std]
#![no_main]

use panic_halt as _;
use rp2040_hal as hal;

mod layout;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

#[rtic::app(device = crate::hal::pac, peripherals = true, dispatchers = [PIO0_IRQ_0])]
mod app {
    use crate::layout::CustomActions;
    use embedded_hal::digital::v2::InputPin;
    use embedded_hal::watchdog::{Watchdog, WatchdogEnable};
    use embedded_time::duration::Extensions;
    use hal::gpio::DynPin;
    use hal::usb::UsbBus;
    use keyberon::debounce::Debouncer;
    use keyberon::key_code;
    use keyberon::layout;
    use keyberon::layout::Layout;
    use keyberon::matrix::{Matrix, PressedKeys};
    use rp2040_hal as hal;
    use usb_device::class_prelude::*;

    const XTAL_FREQ_HZ: u32 = 12_000_000u32;
    const SCAN_TIME_US: u32 = 1000;
    static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<hal::usb::UsbBus>> = None;

    #[shared]
    struct Shared {
        usb_dev: usb_device::device::UsbDevice<'static, hal::usb::UsbBus>,
        usb_class:
            keyberon::hid::HidClass<'static, hal::usb::UsbBus, keyberon::keyboard::Keyboard<()>>,
        uart: hal::pac::UART0,
        timer: hal::timer::Timer,
        alarm: hal::timer::Alarm0,
        #[lock_free]
        matrix: Matrix<DynPin, DynPin, 6, 4>,
        layout: Layout<CustomActions>,
        #[lock_free]
        debouncer: Debouncer<PressedKeys<6, 4>>,
        transform: fn(layout::Event) -> layout::Event,
        #[lock_free]
        watchdog: hal::watchdog::Watchdog,
    }

    #[local]
    struct Local {}

    #[init]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut resets = c.device.RESETS;
        let mut watchdog = hal::watchdog::Watchdog::new(c.device.WATCHDOG);
        watchdog.pause_on_debug(false);

        let clocks = hal::clocks::init_clocks_and_plls(
            XTAL_FREQ_HZ,
            c.device.XOSC,
            c.device.CLOCKS,
            c.device.PLL_SYS,
            c.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        let sio = hal::sio::Sio::new(c.device.SIO);
        let pins = hal::gpio::Pins::new(
            c.device.IO_BANK0,
            c.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        // columns
        let gpio29 = pins.gpio29;
        let gpio28 = pins.gpio28;
        let gpio27 = pins.gpio27;
        let gpio26 = pins.gpio26;
        let gpio22 = pins.gpio22;
        let gpio20 = pins.gpio20;

        // rows
        let gpio4 = pins.gpio4;
        let gpio5 = pins.gpio5;
        let gpio6 = pins.gpio6;
        let gpio7 = pins.gpio7;

        // GPIO0 is high for the right hand side
        let side = pins.gpio21.into_pull_up_input();
        // delay for power on
        for _ in 0..1000 {
            cortex_m::asm::nop();
        }

        // Use a transform to get correct layout from right and left side
        let is_right = side.is_low().unwrap();
        let transform: fn(layout::Event) -> layout::Event = if is_right {
            |e| e.transform(|i: u8, j: u8| -> (u8, u8) { (i, j + 6) })
        } else {
            |e| e.transform(|i: u8, j: u8| -> (u8, u8) { (i, 5 - j) })
        };

        // Enable UART0
        resets.reset.modify(|_, w| w.uart0().clear_bit());
        while resets.reset_done.read().uart0().bit_is_clear() {}
        let uart = c.device.UART0;
        uart.uartibrd.write(|w| unsafe { w.bits(0b0100_0011) });
        uart.uartfbrd.write(|w| unsafe { w.bits(0b0011_0100) });
        uart.uartlcr_h.write(|w| unsafe { w.bits(0b0110_0000) });
        uart.uartcr.write(|w| unsafe { w.bits(0b11_0000_0001) });
        uart.uartimsc.write(|w| w.rxim().set_bit());

        if is_right {
            let _rx_pin = pins.gpio1.into_mode::<hal::gpio::FunctionUart>();
        } else {
            let _tx_pin = pins.gpio0.into_mode::<hal::gpio::FunctionUart>();
        }

        let matrix: Matrix<DynPin, DynPin, 6, 4> = cortex_m::interrupt::free(move |_cs| {
            Matrix::new(
                [
                    // gpio29.into_pull_up_input().into(),
                    // gpio28.into_pull_up_input().into(),
                    // gpio27.into_pull_up_input().into(),
                    // gpio26.into_pull_up_input().into(),
                    // gpio22.into_pull_up_input().into(),
                    // gpio20.into_pull_up_input().into(),
                    gpio20.into_pull_up_input().into(),
                    gpio22.into_pull_up_input().into(),
                    gpio26.into_pull_up_input().into(),
                    gpio27.into_pull_up_input().into(),
                    gpio28.into_pull_up_input().into(),
                    gpio29.into_pull_up_input().into(),
                ],
                [
                    gpio4.into_push_pull_output().into(),
                    gpio5.into_push_pull_output().into(),
                    gpio6.into_push_pull_output().into(),
                    gpio7.into_push_pull_output().into(),
                ],
            )
        })
        .unwrap();

        let layout = Layout::new(crate::layout::LAYERS);
        let debouncer: keyberon::debounce::Debouncer<keyberon::matrix::PressedKeys<6, 4>> =
            Debouncer::new(PressedKeys::default(), PressedKeys::default(), 10);

        let usb_bus = UsbBusAllocator::new(UsbBus::new(
            c.device.USBCTRL_REGS,
            c.device.USBCTRL_DPRAM,
            clocks.usb_clock,
            true,
            &mut resets,
        ));
        unsafe {
            USB_BUS = Some(usb_bus);
        }
        let usb_class = keyberon::new_class(unsafe { USB_BUS.as_ref().unwrap() }, ());
        let usb_dev = keyberon::new_device(unsafe { USB_BUS.as_ref().unwrap() });

        let mut timer = hal::Timer::new(c.device.TIMER, &mut resets);
        let mut alarm = timer.alarm_0().unwrap();
        let _ = alarm.schedule(SCAN_TIME_US.microseconds());
        alarm.enable_interrupt(&mut timer);

        // Start watchdog and feed it with the lowest priority task at 1000hz
        watchdog.start(10_000.microseconds());

        (
            Shared {
                usb_dev,
                usb_class,
                uart,
                timer,
                alarm,
                matrix,
                layout,
                debouncer,
                transform,
                watchdog,
            },
            Local {},
            init::Monotonics(),
        )
    }

    #[task(binds = USBCTRL_IRQ, priority = 3, shared = [usb_dev, usb_class])]
    fn usb_rx(c: usb_rx::Context) {
        let mut usb_d = c.shared.usb_dev;
        let mut usb_c = c.shared.usb_class;
        usb_d.lock(|d| {
            usb_c.lock(|c| {
                if d.poll(&mut [c]) {
                    c.poll();
                }
            })
        });
    }

    #[task(priority = 2, capacity = 8, shared = [usb_dev, usb_class, layout])]
    fn handle_event(mut c: handle_event::Context, event: Option<layout::Event>) {
        match event {
            Some(e) => {
                c.shared.layout.lock(|l| l.event(e));
                return;
            }
            None => match c.shared.layout.lock(|l| l.tick()) {
                layout::CustomEvent::Press(event) => match event {
                    CustomActions::Bootload => {
                        hal::rom_data::reset_to_usb_boot(0, 0);
                    }
                    CustomActions::Reset => {
                        cortex_m::peripheral::SCB::sys_reset();
                    }
                },
                _ => (),
            },
        };
        let report: key_code::KbHidReport = c.shared.layout.lock(|l| l.keycodes().collect());
        if !c
            .shared
            .usb_class
            .lock(|k| k.device_mut().set_keyboard_report(report.clone()))
        {
            return;
        }
        if c.shared.usb_dev.lock(|d| d.state()) != usb_device::device::UsbDeviceState::Configured {
            return;
        }
        while let Ok(0) = c.shared.usb_class.lock(|k| k.write(report.as_bytes())) {}
    }

    #[task(
        binds = TIMER_IRQ_0,
        priority = 1,
        shared = [uart, matrix, debouncer, timer, alarm, layout, &transform, watchdog],
    )]
    fn scan_timer_irq(mut c: scan_timer_irq::Context) {
        let timer = c.shared.timer;
        let alarm = c.shared.alarm;
        (timer, alarm).lock(|t, a| {
            a.clear_interrupt(t);
            let _ = a.schedule(SCAN_TIME_US.microseconds());
        });

        c.shared.watchdog.feed();

        for event in c
            .shared
            .debouncer
            .events(c.shared.matrix.get().unwrap())
            .map(c.shared.transform)
        {
            let mut byte: u8;
            byte = event.coord().1;
            byte |= (event.coord().0 & 0b0000_0111) << 4;
            byte |= (event.is_press() as u8) << 7;
            // Watchdog will catch any possibility for an infinite loop
            while c.shared.uart.lock(|u| u.uartfr.read().txff().bit_is_set()) {}
            c.shared
                .uart
                .lock(|u| u.uartdr.write(|w| unsafe { w.data().bits(byte) }));
            handle_event::spawn(Some(event)).unwrap();
        }
        handle_event::spawn(None).unwrap();
    }

    #[task(binds = UART0_IRQ, priority = 4, shared = [uart])]
    fn rx(mut c: rx::Context) {
        // RX FIFO is disabled so we just check that the byte received is valid
        // and then we read it. If a bad byte is received, it is possible that the
        // receiving side will never read. TODO: fix this
        if c.shared.uart.lock(|u| {
            u.uartmis.read().rxmis().bit_is_set()
                && u.uartfr.read().rxfe().bit_is_clear()
                && u.uartdr.read().oe().bit_is_clear()
                && u.uartdr.read().be().bit_is_clear()
                && u.uartdr.read().pe().bit_is_clear()
                && u.uartdr.read().fe().bit_is_clear()
        }) {
            let d: u8 = c.shared.uart.lock(|u| u.uartdr.read().data().bits());
            if (d & 0b10000000) > 0 {
                handle_event::spawn(Some(layout::Event::Press(
                    (d >> 4) & 0b0000_0111,
                    d & 0b0000_1111,
                )))
                .unwrap();
            } else {
                handle_event::spawn(Some(layout::Event::Release(
                    (d >> 4) & 0b0000_0111,
                    d & 0b0000_1111,
                )))
                .unwrap();
            }
        }
    }
}
