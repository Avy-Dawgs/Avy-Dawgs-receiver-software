use stm32g4xx_hal::{gpio::{self}, pac, opamp::{InternalOutput, Opamp1, Opamp2, Pga}};

pub enum OpampGainDB
{
    Gain6, 
    Gain12, 
    Gain18, 
    Gain24,
}

pub trait OpampExt
{
    fn set_gain(&mut self, gain: OpampGainDB);
}

macro_rules! pga_set_gain {
    ($csr:ident, $pga:ty) => {
        

        impl OpampExt for $pga
        {
            fn set_gain(&mut self, gain: OpampGainDB)
            {
                use pac::opamp::$csr::PGA_GAIN as Gain;
                unsafe { (*pac::OPAMP::ptr()) 
                    .$csr()
                    .write(|reg| 
                        reg.pga_gain()
                        .variant(
                            match gain {
                            OpampGainDB::Gain6 => {
                                Gain::Gain2
                            },
                            OpampGainDB::Gain12 => {
                                Gain::Gain4
                            }, 
                            OpampGainDB::Gain18 => {
                                Gain::Gain8
                            },
                            OpampGainDB::Gain24 => {
                                Gain::Gain16
                            },
                        })
                    );
                };
            }
        }
    };
}


pga_set_gain!(opamp1_csr, Pga<Opamp1, gpio::Pin<'A', 1>, InternalOutput>);
pga_set_gain!(opamp2_csr, Pga<Opamp2, gpio::Pin<'A', 7>, InternalOutput>);
