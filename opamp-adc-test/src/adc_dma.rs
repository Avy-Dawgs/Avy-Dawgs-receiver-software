// use crate::Buffer;
use crate::BUFFER_SIZE;

use stm32g4xx_hal as hal;

use stm32g4xx_hal::{adc::{Disabled, DMA, Adc}, dma::{channel::C, traits::{Channel, TargetAddress}}};

use core::{mem, sync::atomic::fence};

pub struct AdcDmaTransfer<A, D> 
where 
    A: hal::adc::Instance,
    D: hal::dma::channel::Instance,
{
    adc: Option<Adc<A, DMA>>, 
    channel: C<D, 0>, 
    buffer: &'static mut [u16; BUFFER_SIZE]
}

impl<A, D> AdcDmaTransfer<A, D> 
where 
    A: hal::adc::Instance,
    D: hal::dma::channel::Instance
{
    /// Configure and start the first transfer.
    pub fn init(mut adc: Adc<A, Disabled>, mut channel: C<D, 0>, buffer: &'static mut [u16; BUFFER_SIZE]) -> Self
    {
        adc.reset_sequence(); 
        unsafe 
        {
            (*hal::pac::ADC1::ptr()) 
                .sqr1() 
                .modify(|_, w| 
                    w.sq1().bits(1) 
                );
            
        }
        adc.set_continuous(hal::adc::config::Continuous::Continuous);
        let mut adc_dma = adc.enable_dma(hal::adc::config::Dma::Single);

        channel.disable(); 
        fence(core::sync::atomic::Ordering::SeqCst);

        unsafe 
        {
            channel.set_peripheral_address(adc_dma.address());
            channel.set_memory_address(buffer.as_ptr() as u32);
            channel.set_number_of_transfers(BUFFER_SIZE as u16);
            channel.set_priority(stm32g4xx_hal::dma::config::Priority::Medium);
            channel.set_direction(stm32g4xx_hal::dma::DmaDirection::PeripheralToMemory);
            channel.set_circular_buffer(false);
            channel.set_peripheral_increment(false);
            channel.set_memory_increment(true);
            channel.set_peripheral_size(1);
            channel.set_memory_size(1);
            channel.set_half_transfer_interrupt_enable(false);
            channel.set_transfer_error_interrupt_enable(false);
            channel.set_transfer_complete_interrupt_enable(false);
            channel.set_request_line(5);
            fence(core::sync::atomic::Ordering::SeqCst);
            channel.enable();
        }
        adc_dma.start_conversion();

        Self {adc: Some(adc_dma), channel: channel, buffer: buffer}
    }

    /// Wait for the current transfer to end, swap the current buffer with a new one, and restart
    /// the transfer.
    pub fn wait_and_swap_buffers(&mut self, buffer: &'static mut [u16; BUFFER_SIZE]) -> &'static mut [u16; BUFFER_SIZE]
    {

        while !self.channel.get_transfer_complete_flag() 
        {

        }
        self.channel.clear_interrupts();

        let mut adc = self.adc.take().expect("");
        
        self.channel.disable();
        fence(core::sync::atomic::Ordering::SeqCst);

        unsafe 
        {
            self.channel.set_memory_address(buffer.as_ptr() as u32);
            self.channel.set_number_of_transfers(BUFFER_SIZE as u16);
            fence(core::sync::atomic::Ordering::SeqCst);
            self.channel.enable();
        }
        adc.start_conversion();
        self.adc = Some(adc);

        mem::replace(&mut self.buffer, buffer)
    }
}
