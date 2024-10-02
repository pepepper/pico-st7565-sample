#![no_main]
#![no_std]

use defmt_rtt as _;
use rp_pico as _;
use panic_probe as _;

#[defmt::panic_handler]
fn panic() -> ! {
	cortex_m::asm::udf()
}