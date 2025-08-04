#![no_main] 
#![no_std]

use stm32g4xx_hal as hal;

use hal::{stm32,
    adc::{config::{self, SampleTime, Sequence}, AdcClaim, AdcCommonExt}, cortex_m::{self}, dma::{channel::DMAExt}, opamp::{OpampEx, Gain, IntoPga}, pwr::{PwrExt}, rcc::{self, RccExt}, 
    gpio::GpioExt, delay::SYSTDelayExt,
};

use cortex_m_rt::{entry};
use panic_probe as _;
use defmt_rtt as _;
use defmt::{info, println};

mod amplifier;
mod adc_dma;
use crate::{adc_dma::AdcDmaTransfer, amplifier::{OpampExt, OpampGainDB}}; 


const BUFFER_SIZE: usize = 100;
type Buffer = [u16; BUFFER_SIZE];

#[entry]
fn main() -> ! 
{
    info!("Program Starting");

    let dp = stm32::Peripherals::take().unwrap();       // device peripherals
    let cp = cortex_m::Peripherals::take().unwrap();    // core peripherals

    // configure clocks
    let mut rcc = config_clks(dp.RCC, dp.PWR);
    let mut delay = cp.SYST.delay(&rcc.clocks);

    info!("sys clock freq: {} Hz", rcc.clocks.sys_clk);
    info!("core clock freq: {} Hz", rcc.clocks.core_clk);

    // configure led
    let gpiob = dp.GPIOB.split(&mut rcc);
    let mut led = gpiob.pb8.into_push_pull_output();

    let gpioa = dp.GPIOA.split(&mut rcc);
    let pa0 = gpioa.pa0.into_analog();
    let pa1 = gpioa.pa1.into_analog();
    let pa3 = gpioa.pa3.into_analog();

    // dma
    let channels = dp.DMA1.split(&rcc);

    let ch1 = channels.ch1;

    // configure opamps
    let (opamp1, _opamp2, _opamp3) = dp.OPAMP.split(&mut rcc);
    let mut pga1 = opamp1.pga_external_bias(pa1, pa3, Gain::Gain2);
    pga1.set_gain(OpampGainDB::Gain6);

    // configure adcs
    let clk_cfg = config::ClockMode::AdcKerCk { prescaler: (config::Prescaler::Div_1), src: (config::ClockSource::PllP) };
    let adc12_common = dp.ADC12_COMMON.claim(clk_cfg, &mut rcc);
    let mut adc = adc12_common.claim(dp.ADC1, &mut delay);

    adc.reset_sequence(); 
    adc.configure_channel(&pa0, Sequence::One, SampleTime::Cycles_2_5);

    let mut aux_buffer = cortex_m::singleton!(BUF2: Buffer = [0; BUFFER_SIZE]).unwrap();
    let first_buffer = cortex_m::singleton!(BUF1 : Buffer = [0; BUFFER_SIZE]).unwrap();

    let mut transfer = AdcDmaTransfer::init(adc, ch1, first_buffer);

    loop {
        // wait for next buffer
        let buffer = transfer.wait_and_swap_buffers(aux_buffer);

        // process
        let max_val = max(buffer);
        let avg_val = avg(buffer);
        println!("max value: {}", max_val);
        println!("avg value: {}", avg_val);

        // assign auxillary buffer
        aux_buffer = buffer;
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

fn avg(list: &[u16]) -> u16 
{
    let mut sum: u32 = 0;
    for i in 0..list.len() 
    {
        sum += list[i] as u32;
    }
    ((sum as f32) / (list.len() as f32)) as u16
}

/// Configures clocks. 
#[inline]
fn config_clks(rcc: stm32::RCC, pwr: stm32::PWR) -> rcc::Rcc
{
    // pll config 
    let mut pll_cfg = rcc::PllConfig::default();
    pll_cfg.n = rcc::PllNMul::MUL_85; 
    pll_cfg.m = rcc::PllMDiv::DIV_4;
    pll_cfg.r = Some(rcc::PllRDiv::DIV_2);
    pll_cfg.q = Some(rcc::PllQDiv::DIV_2); 
    pll_cfg.p = Some(rcc::PllPDiv::DIV_6);

    // clock config object
    let clk_cfg = rcc::Config::pll().pll_cfg(pll_cfg).boost(true);
    let clk_cfg = clk_cfg.pll_cfg(pll_cfg);             

    // apply config
    let pwr = pwr.constrain().vos(hal::pwr::VoltageScale::Range1{ enable_boost: true }).freeze();
    rcc.constrain().freeze(clk_cfg, pwr)
}
