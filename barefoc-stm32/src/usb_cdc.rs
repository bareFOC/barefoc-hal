//! Blocking `embedded_io` streams over an embassy-usb CDC ACM class.
//!
//! Enables the device-side `barefoc-passthrough` server to use the same
//! protocol bytes over native USB as over a USART CDC link. The USB device and
//! `CdcAcmClass` are created by the application; this module only supplies the
//! [`embedded_io::Read`] / [`embedded_io::Write`] bridges obtained from
//! `CdcAcmClass::split()`.
//!
//! Passthrough is a configuration/diagnostics path, not a real-time control
//! loop. `block_on` is used deliberately to adapt embassy's async endpoints to
//! the blocking `embedded_io` traits expected by the server; run the server in
//! its own task and do not use these bridges inside the FOC control loop.
//!
//! Upstream: `barefoc-comms/barefoc-multibus/src/transport.rs` (single task
//! owns bus I/O), `barefoc-hal/barefoc-stm32/src/lib.rs` (feature conventions),
//! `openspec/changes/bus-passthrough/design.md` (decision 7).

use core::cmp;

use embassy_usb::class::cdc_acm::{Receiver, Sender};
use embassy_usb::driver::{Driver, EndpointError};
use embedded_io::{ErrorKind, ErrorType, Read, Write};

/// Error reported by the CDC stream bridges.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UsbCdcError {
    /// Whether the USB CDC link was disconnected.
    pub disconnected: bool,
}

impl embedded_io::Error for UsbCdcError {
    fn kind(&self) -> ErrorKind {
        if self.disconnected {
            ErrorKind::NotConnected
        } else {
            ErrorKind::Other
        }
    }
}

fn map_endpoint(error: EndpointError) -> UsbCdcError {
    match error {
        EndpointError::Disabled => UsbCdcError { disconnected: true },
        EndpointError::BufferOverflow => UsbCdcError {
            disconnected: false,
        },
    }
}

/// Blocking read half of a CDC ACM class, for `PassthroughServer`'s reader.
pub struct UsbCdcReader<'d, D: Driver<'d>> {
    receiver: Receiver<'d, D>,
}

impl<'d, D: Driver<'d>> UsbCdcReader<'d, D> {
    /// Wrap the CDC ACM receiver.
    pub const fn new(receiver: Receiver<'d, D>) -> Self {
        Self { receiver }
    }

    /// Borrow the CDC receiver.
    pub const fn receiver(&self) -> &Receiver<'d, D> {
        &self.receiver
    }

    /// Wait until the USB host has enabled the CDC interface.
    pub async fn wait_connection(&mut self) {
        self.receiver.wait_connection().await;
    }
}

impl<'d, D: Driver<'d>> ErrorType for UsbCdcReader<'d, D> {
    type Error = UsbCdcError;
}

impl<'d, D: Driver<'d>> Read for UsbCdcReader<'d, D> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        embassy_futures::block_on(self.receiver.read_packet(buf)).map_err(map_endpoint)
    }
}

/// Blocking write half of a CDC ACM class, for `PassthroughServer`'s writer.
pub struct UsbCdcWriter<'d, D: Driver<'d>> {
    sender: Sender<'d, D>,
    needs_zlp: bool,
}

impl<'d, D: Driver<'d>> UsbCdcWriter<'d, D> {
    /// Wrap the CDC ACM sender.
    pub const fn new(sender: Sender<'d, D>) -> Self {
        Self {
            sender,
            needs_zlp: false,
        }
    }

    /// Borrow the CDC sender.
    pub const fn sender(&self) -> &Sender<'d, D> {
        &self.sender
    }

    /// Wait until the USB host has enabled the CDC interface.
    pub async fn wait_connection(&mut self) {
        self.sender.wait_connection().await;
    }
}

impl<'d, D: Driver<'d>> ErrorType for UsbCdcWriter<'d, D> {
    type Error = UsbCdcError;
}

impl<'d, D: Driver<'d>> Write for UsbCdcWriter<'d, D> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        let len = cmp::min(buf.len(), self.sender.max_packet_size() as usize);
        embassy_futures::block_on(self.sender.write_packet(&buf[..len])).map_err(map_endpoint)?;
        self.needs_zlp = len == self.sender.max_packet_size() as usize;
        Ok(len)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        if self.needs_zlp {
            embassy_futures::block_on(self.sender.write_packet(&[])).map_err(map_endpoint)?;
            self.needs_zlp = false;
        }
        Ok(())
    }
}

/// Combined blocking stream, useful for echo-style applications.
///
/// For `PassthroughServer`, prefer [`UsbCdcStream::split`] so the reader and
/// writer can be owned independently.
pub struct UsbCdcStream<'d, D: Driver<'d>> {
    reader: UsbCdcReader<'d, D>,
    writer: UsbCdcWriter<'d, D>,
}

impl<'d, D: Driver<'d>> UsbCdcStream<'d, D> {
    /// Wrap a CDC ACM receiver and sender pair.
    pub const fn new(receiver: Receiver<'d, D>, sender: Sender<'d, D>) -> Self {
        Self {
            reader: UsbCdcReader::new(receiver),
            writer: UsbCdcWriter::new(sender),
        }
    }

    /// Split into independent blocking reader and writer halves.
    pub fn split(self) -> (UsbCdcReader<'d, D>, UsbCdcWriter<'d, D>) {
        (self.reader, self.writer)
    }

    /// Wait until the USB host has enabled the CDC interface.
    pub async fn wait_connection(&mut self) {
        self.reader.wait_connection().await;
        self.writer.wait_connection().await;
    }
}

impl<'d, D: Driver<'d>> ErrorType for UsbCdcStream<'d, D> {
    type Error = UsbCdcError;
}

impl<'d, D: Driver<'d>> Read for UsbCdcStream<'d, D> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.reader.read(buf)
    }
}

impl<'d, D: Driver<'d>> Write for UsbCdcStream<'d, D> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.writer.write(buf)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.writer.flush()
    }
}
