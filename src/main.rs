#![no_main]
#![no_std]

use display_interface_spi::SPIInterface;
use embedded_graphics::{
    pixelcolor::BinaryColor,
    prelude::{Point, Primitive},
    primitives::{Circle, PrimitiveStyle},
    Drawable,
};
use embedded_hal::{digital::v2::OutputPin, 
    spi::MODE_0}
;

use rp_pico::{
    entry, hal::{
        self, fugit::RateExtU32, gpio::{self}, Clock, Sio, Spi
    }, pac, XOSC_CRYSTAL_FREQ
};

use pico_st7565_sample as _;
use st7565::{
    types::{BoosterRatio, PowerControlMode},
    DisplaySpecs, GraphicsPageBuffer, ST7565,
};

struct WOM12864J5;
impl DisplaySpecs<128, 64, 8> for WOM12864J5 {
    const FLIP_ROWS: bool = false;
    const FLIP_COLUMNS: bool = false;
    const INVERTED: bool = false;
    const BIAS_MODE_1: bool = true;
    const POWER_CONTROL: PowerControlMode = PowerControlMode {
        booster_circuit: true,
        voltage_regulator_circuit: true,
        voltage_follower_circuit: true,
    };
    const VOLTAGE_REGULATOR_RESISTOR_RATIO: u8 = 0b011;
    const ELECTRONIC_VOLUME: u8 = 0b011111;
    const BOOSTER_RATIO: BoosterRatio = BoosterRatio::StepUp2x3x4x;
    const COLUMN_OFFSET: u8 = 0;
}

#[entry]
fn entry() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let clocks = hal::clocks::init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
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

    let sio = Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut reset = pins.gpio1.into_push_pull_output();
    let spi_cs = pins.gpio0.into_push_pull_output();
    let _spi_sclk = pins.gpio2.into_function::<gpio::FunctionSpi>(); // scl
    let _spi_mosi = pins.gpio3.into_function::<gpio::FunctionSpi>(); // sda
    let spi_dc = pins.gpio4.into_push_pull_output();
    let disp_vcc = pins.gpio5.into_push_pull_output().set_high();
    // Create an SPI driver instance for the SPI0 device
    let spi_device = pac.SPI0;
    let spi_pin_layout = (_spi_mosi, _spi_sclk);

    let spi = Spi::<_, _, _, 8>::new(spi_device, spi_pin_layout).init(
        &mut pac.RESETS,
        125_000_000u32.Hz(),
        16_000_000u32.Hz(),
        MODE_0,
    );
    let disp_spi = SPIInterface::new(spi, spi_dc, spi_cs);

    let mut page_buffer = GraphicsPageBuffer::new();
    let mut disp = ST7565::new(disp_spi, WOM12864J5).into_graphics_mode(&mut page_buffer);
    disp.reset(&mut reset, &mut delay).unwrap();
    disp.flush().unwrap();
    disp.set_display_on(true).unwrap();
    Circle::new(Point::new(10, 6), 20)
        .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 2))
        .draw(&mut disp)
        .unwrap();
    disp.flush().unwrap();
    loop {}
}
