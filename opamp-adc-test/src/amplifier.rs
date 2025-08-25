use stm32g4xx_hal::{self as hal, dac::DacOutSet, delay::Delay, gpio::Analog, opamp::Gain, rcc::Rcc};
use hal::{gpio::{self}, pac, opamp::{InternalOutput, Opamp1, Opamp2, Pga, Disabled}, delay::SYSTDelayExt, prelude::DelayNs};


type DacChannel = hal::dac::DacCh<hal::stm32g4::Periph<hal::pac::dac3::RegisterBlock, 1342181376>, 0, {hal::dac::M_INT_SIG}, hal::dac::Disabled>;

// pub enum OpampGainDB
// {
//     Gain6, 
//     Gain12, 
//     Gain18, 
//     Gain24,
// }

#[derive(Clone, Copy, PartialEq)]
pub enum OpampGainDB2
{
    Gain0, 
    Gain10, 
    Gain17, 
    Gain24,
}

// pub trait OpampPga
// {
//     fn set_gain(&mut self, gain: OpampGainDB);
//
// }

pub struct OpampPga2 
{
    gain: OpampGainDB2,
    bias: u16,
}

impl OpampPga2
{
    pub fn init(opamp: Disabled<Opamp1>, dac: DacChannel, vinm0: gpio::Pin<'A', 3, Analog>, rcc: &mut Rcc) -> Self
    {
        let _ = opamp;
        let _ = vinm0;
        let mut dac_en = dac.enable(rcc); 
        let bias = 2047;
        dac_en.set_value(bias);
    
        // configure opamp
        unsafe {
            (*pac::OPAMP::ptr()) 
                .opamp1_csr() 
                .write(|w| 
                    // select inverting input as pga
                    w.vm_sel().variant(pac::opamp::opamp1_csr::VM_SEL::Pga)  
                    // non-inverting input select dac 
                    .vp_sel().variant(pac::opamp::opamp1_csr::VP_SEL::Dac3Ch1)
                    // output to adc
                    .opaintoen().variant(pac::opamp::opamp1_csr::OPAINTOEN::Adcchannel) 
                    // set the gain to -1, take input from Vinm0
                    .pga_gain().variant(pac::opamp::opamp1_csr::PGA_GAIN::Gain2InputVinm0)
                    // enable opamp
                    .opaen().variant(pac::opamp::opamp1_csr::OPAEN::Enabled)
                );
        };



        Self { gain: OpampGainDB2::Gain0, bias: bias}
    }

    pub fn set_gain(&mut self, gain: OpampGainDB2)
    {
        if gain == self.gain
        {
            return;
        }
        self.gain = gain;

        unsafe 
        {
            (*pac::OPAMP::ptr()) 
                .opamp1_csr() 
                .write(|w| 
                    w.pga_gain().variant( 
                        match gain 
                        {
                            OpampGainDB2::Gain0 => pac::opamp::opamp1_csr::PGA_GAIN::Gain2InputVinm0, 
                            OpampGainDB2::Gain10 => pac::opamp::opamp1_csr::PGA_GAIN::Gain4InputVinm0, 
                            OpampGainDB2::Gain17 => pac::opamp::opamp1_csr::PGA_GAIN::Gain8InputVinm0, 
                            OpampGainDB2::Gain24 => pac::opamp::opamp1_csr::PGA_GAIN::Gain16InputVinm0,
                        }
                    )
                );
        };
    }

    pub fn get_gain(&self) -> OpampGainDB2 
    {
        self.gain
    }

    pub fn get_bias(&self) -> u16 
    {
        self.bias
    }

}

// macro_rules! pga_set_gain {
//     ($csr:ident, $pga:ty) => {
//
//
//         impl OpampPga for $pga
//         {
//             fn set_gain(&mut self, gain: OpampGainDB)
//             {
//                 use pac::opamp::$csr::PGA_GAIN as Gain;
//                 unsafe { (*pac::OPAMP::ptr()) 
//                     .$csr()
//                     .write(|reg| 
//                         reg.pga_gain()
//                         .variant(
//                             match gain {
//                                 OpampGainDB::Gain6 => 
//                                 {
//                                     Gain::Gain2
//                                 },
//                                 OpampGainDB::Gain12 => 
//                                 {
//                                     Gain::Gain4
//                                 }, 
//                                 OpampGainDB::Gain18 => 
//                                 {
//                                     Gain::Gain8
//                                 },
//                                 OpampGainDB::Gain24 => 
//                                 {
//                                     Gain::Gain16
//                                 },
//                             }
//                         )
//                     );
//                 };
//             }
//         }
//     };
// }
//
//
// pga_set_gain!(opamp1_csr, Pga<Opamp1, gpio::Pin<'A', 1>, InternalOutput>);
// pga_set_gain!(opamp2_csr, Pga<Opamp2, gpio::Pin<'A', 7>, InternalOutput>);
