use mcp251xfd::registers::ram::FifoLayout;
use mcp251xfd::registers::{Fifo, PayloadSize};

// 72 bytes/element * 29 = 2088 > 2048: must fail to compile.
const LAYOUT: FifoLayout = FifoLayout::new().rx_fifo(Fifo::F1, PayloadSize::B64, 29);

fn main() {
    let _ = LAYOUT;
}
