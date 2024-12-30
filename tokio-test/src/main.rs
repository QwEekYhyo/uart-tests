use tokio::io::AsyncReadExt;
use tokio_serial::SerialPortBuilderExt;
use tokio::task;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let device = "/dev/ttyAMA0";
    let baud_rate = 115_200;

    let mut port = tokio_serial::new(device, baud_rate)
        .data_bits(tokio_serial::DataBits::Eight)
        .stop_bits(tokio_serial::StopBits::One)
        .parity(tokio_serial::Parity::None)
        .flow_control(tokio_serial::FlowControl::None)
        .open_native_async()?;

    /* I need to call the C function tcflush somehow here */

    let reader_handle = task::spawn(async move {
        let mut buffer = [0u8; 256];
        loop {
            match port.read(&mut buffer).await {
                Ok(bytes_read) if bytes_read > 0 => {
                    let received_data = String::from_utf8_lossy(&buffer[..bytes_read]);
                    println!("Data received: {}", received_data);
                }
                Ok(_) => {println!("No data");}
                Err(e) => {
                    eprintln!("Error reading from serial port: {}", e);
                    break;
                }
            }
        }
    });

    let _ = tokio::join!(reader_handle);

    Ok(())
}
