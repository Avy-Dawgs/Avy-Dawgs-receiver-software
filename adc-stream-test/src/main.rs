#![no_main] 
#![no_std]

use stm32g4xx_hal::{
    self as hal, adc::{self, config::Dma as AdcDma, AdcClaim, AdcCommonExt}, cortex_m::{self, prelude}, delay::SYSTDelayExt, dma::{channel::DMAExt, config::DmaConfig, TransferExt}, gpio::{self, *}, prelude::*, pwr::PwrExt, rcc::{self, PllConfig, PllSrc, RccExt}, stm32, pwr,
};

use cortex_m_rt::entry;
use panic_halt as _;
use defmt_rtt as _;
use defmt::{info, println}; 

#[entry]
fn main() -> ! 
{
    info!("Program Starting");

    let dp = stm32::Peripherals::take().unwrap();       // device peripherals
    let cp = cortex_m::Peripherals::take().unwrap();    // core peripherals

    // configure clocks
    let mut rcc = config_clks(dp.RCC, dp.PWR);

    let mut delay = cp.SYST.delay(&rcc.clocks);

    // configure led
    let gpiob = dp.GPIOB.split(&mut rcc);
    let mut led = gpiob.pb8.into_push_pull_output();

    info!("sys clock freq: {} Hz", rcc.clocks.sys_clk);
    info!("core clock freq: {} Hz", rcc.clocks.core_clk);

    // GPIO CONFIG 
    let gpioa = dp.GPIOA.split(&mut rcc); 
    let pa0 = gpioa.pa0.into_analog();

    // ADC CONFIG 
    let adc12_common = dp.ADC12_COMMON.claim(adc::config::ClockMode::AdcHclkDiv4, &mut rcc);
    let mut adc = adc12_common.claim(dp.ADC1, &mut delay);


    adc.set_continuous(adc::config::Continuous::Continuous);
    adc.reset_sequence(); 
    adc.configure_channel(&pa0, adc::config::Sequence::One, adc::config::SampleTime::Cycles_640_5);


    // DMA CONFIG 
    let channels = dp.DMA1.split(&rcc);
    let config = DmaConfig::default() 
        .transfer_complete_interrupt(false) 
        .circular_buffer(true)  
        .memory_increment(true);

    let buffer = cortex_m::singleton!(: [u16; 2048] = [0; 2048]).unwrap();
    let mut transfer = channels.ch1.into_circ_peripheral_to_memory_transfer( 
        adc.enable_dma(AdcDma::Continuous), 
        &mut buffer[..], 
        config, 
        );

    transfer.start(|adc| adc.start_conversion());

    delay.delay_ms(500);
    let (channel, adc, buffer) = transfer.free();
    loop {
        led.set_low(); 
        delay.delay_ms(500);
        led.set_high();

        let max = max(buffer);

        println!("ADC max: {}", max);

    }
}

fn max(list: &[u16]) -> u16
{
    let mut max: u16 = 0;
    for i in 0..list.len() {
        if list[i] > max {
            max = list[i];
        }
    }
    max
}

/// Configures clocks. 
#[inline]
fn config_clks(rcc: stm32::RCC, pwr: stm32::PWR) -> rcc::Rcc
{
    // pll config 
    let mut pll_cfg = rcc::PllConfig::default();
    pll_cfg.mux = PllSrc::HSI;
    pll_cfg.n = rcc::PllNMul::MUL_85; 
    pll_cfg.m = rcc::PllMDiv::DIV_4;
    pll_cfg.r = Some(rcc::PllRDiv::DIV_2);
    pll_cfg.q = Some(rcc::PllQDiv::DIV_2); 
    pll_cfg.p = Some(rcc::PllPDiv::DIV_2);

    // clock config object
    let clk_cfg = rcc::Config::pll().pll_cfg(pll_cfg).boost(true);

    // apply config
    let pwr = pwr.constrain().vos(pwr::VoltageScale::Range1{ enable_boost: true }).freeze();

    rcc.constrain().freeze(clk_cfg, pwr)
}
