#![no_main] 
#![no_std]

use stm32g4xx_hal::{
    self as hal, adc::{config::{self, Continuous, SampleTime, Sequence}, Adc, AdcClaim, AdcCommonExt, DMA}, cortex_m::{self, asm::delay}, dma::{self, channel::{DMAExt, C}, config::DmaConfig, transfer::MutTransfer, PeripheralToMemory, Transfer, TransferExt}, gpio::{self, *}, opamp::Gain, pac::{GPIOB}, prelude::*, pwr, rcc::{self, RccExt}, stm32::{self, interrupt, Interrupt}, stm32g4::{self, Periph}
};

use cortex_m_rt::{entry};
use core::cell::RefCell;
use cortex_m::{asm::wfi, interrupt::Mutex};
use panic_probe as _;
use defmt_rtt as _;
use defmt::{info, println};

mod amplifier;
use crate::amplifier::{OpampExt, OpampGainDB}; 

type Adc1Transfer = Transfer<C<Periph<crate::hal::pac::dma1::RegisterBlock, 1073872896>, 0>, Adc<Periph<hal::pac::adc1::RegisterBlock, 1342177280>, DMA>, PeripheralToMemory, &'static mut [u16], MutTransfer>;
type Buffer = [u16; BUFFER_SIZE];

const BUFFER_SIZE: usize = 7000;

static TRANSFER: Mutex<RefCell<Option<Adc1Transfer>>> = Mutex::new(RefCell::new(None));
static mut BUFFER1: Buffer = [0; BUFFER_SIZE];
static mut BUFFER2: Buffer = [0; BUFFER_SIZE];

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
    // let gpioc = dp.GPIOC.split(&mut rcc);
    let mut led = gpiob.pb8.into_push_pull_output();

    let gpioa = dp.GPIOA.split(&mut rcc);
    let pa1 = gpioa.pa1.into_analog();
    let pa5 = gpioa.pa5.into_analog();
    let pa3 = gpioa.pa3.into_analog();
    let pa7 = gpioa.pa7.into_analog();

    // dma
    let channels = dp.DMA1.split(&rcc);
    let dma_config = DmaConfig::default()
        .transfer_complete_interrupt(false)
        .circular_buffer(false)
        .memory_increment(true);

    // configure opamps
    let (opamp1, opamp2, _opamp3) = dp.OPAMP.split(&mut rcc);
    let mut pga1 = opamp1.pga_external_bias(pa1, pa3, Gain::Gain2);
    let mut pga2 = opamp2.pga_external_bias(pa7, pa5, Gain::Gain2);
    pga1.set_gain(OpampGainDB::Gain6);
    pga2.set_gain(OpampGainDB::Gain6);

    // configure adcs
    let clk_cfg = config::ClockMode::AdcKerCk { prescaler: (config::Prescaler::Div_1), src: (config::ClockSource::PllP) };
    let adc12_common = dp.ADC12_COMMON.claim(clk_cfg, &mut rcc);
    let mut adc1 = adc12_common.claim(dp.ADC1, &mut delay);
    let mut adc2 = adc12_common.claim(dp.ADC2, &mut delay);
    adc1.set_continuous(Continuous::Continuous);
    adc1.reset_sequence(); 
    adc1.configure_channel(&pga1, Sequence::One, SampleTime::Cycles_2_5);

    adc2.set_continuous(Continuous::Continuous);
    adc2.reset_sequence();
    adc2.configure_channel(&pga2, Sequence::One, SampleTime::Cycles_2_5);

    // let mut first_buffer = cortex_m::singleton!(: [u16; 10000] = [0; 10000]).unwrap();

    // cortex_m::interrupt::free(|cs| *TRANSFER.borrow(cs).borrow_mut() = Some(transfer));

    let mut ch1 = channels.ch1;

    let mut buf1_active = true;

    // MAIN LOOP 
    loop 
    {

        let mut transfer = match buf1_active {
            true => {
                unsafe {
                    ch1.into_peripheral_to_memory_transfer(
                        adc1.enable_dma(config::Dma::Single),
                        &mut BUFFER1[..],
                        dma_config,
                    )
                } 
            },
            false => {
                unsafe {
                    ch1.into_peripheral_to_memory_transfer(
                        adc1.enable_dma(config::Dma::Single),
                        &mut BUFFER2[..],
                        dma_config,
                    )
                }
            },
        };

        transfer.start(|adc| adc.start_conversion());

        info!("waiting for transfer");
        while !transfer.get_transfer_complete_flag() { } 
        info!("done with transfer");

        // let max = transfer.peek_buffer(|buf, size| max(buf));
        // println!("max: {}", max);

        let (ch1_, adc1_, _) = transfer.free();
        ch1 = ch1_; 
        adc1 = adc1_.disable();

        let max = match buf1_active {
            true => unsafe {max(&BUFFER1[..])}, 
            false => unsafe {max(&BUFFER2[..])} 
        };
        println!("max: {}", max);

        // let sample1 = adc1.convert(&pga1, SampleTime::Cycles_640_5);
        // let sample2 = adc2.convert(&pga2, SampleTime::Cycles_640_5);
        // info!{"adc1 sample: {}", sample1};
        // info!{"adc2 sample: {}", sample2};
        led.toggle();
        delay.delay_ms(100);
        buf1_active = !buf1_active;
    }
}

fn max(list: &'static [u16]) -> u16
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
    pll_cfg.n = rcc::PllNMul::MUL_85; 
    pll_cfg.m = rcc::PllMDiv::DIV_4;
    pll_cfg.r = Some(rcc::PllRDiv::DIV_2);
    pll_cfg.q = Some(rcc::PllQDiv::DIV_2); 
    pll_cfg.p = Some(rcc::PllPDiv::DIV_6);

    // clock config object
    let clk_cfg = rcc::Config::pll().pll_cfg(pll_cfg).boost(true);
    let clk_cfg = clk_cfg.pll_cfg(pll_cfg);             

    // apply config
    let pwr = pwr.constrain().vos(pwr::VoltageScale::Range1{ enable_boost: true }).freeze();
    rcc.constrain().freeze(clk_cfg, pwr)
}

#[interrupt] 
fn DMA1_CH1()
{

}
