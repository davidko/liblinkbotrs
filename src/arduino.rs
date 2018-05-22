
pub enum PinMode {
    Input = 0,
    Output = 1,
    InputPullup = 2,
}

pub fn pin_mode_from_u8(value: u8) -> Result<PinMode, String> {
    match value {
        0 => Ok(PinMode::Input),
        1 => Ok(PinMode::Output),
        2 => Ok(PinMode::InputPullup),
        other => Err(format!("Unknown pin mode: {}", other)),
    }
}

pub trait ArduinoLinkbot {
    /// Set duty-cycle of analog pin.
    fn analog_write(&mut self, pin: u8, value: u8) -> Result<(), String>;

    /// Read from ADC pin
    fn analog_read(&mut self, pin: u8) -> Result<u16, String>;

    /// Set the value of a GPIO pin. 0 sets the pin low, any other value sets the pin high.
    fn digital_write(&mut self, pin: u8, value: u8) -> Result<(), String>;

    /// Read the value from digital GPIO pin. Returns 0 or 1 on success.
    fn digital_read(&mut self, pin: u8) -> Result<u8, String>;

    /// Set an Arduino pin's mode
    fn set_pin_mode(&mut self, pin: u8, mode: PinMode) -> Result<(), String>;
}

enum Command {
    Header = 0x22,
    PinMode = 0x02,
    DigitalWrite = 0x03,
    DigitalRead = 0x04,
    AnalogWrite = 0x05,
    AnalogRead = 0x06,
    // AnalogRef = 0x07,
}

const TWI_ADDR:u32 = 0x02;
impl ArduinoLinkbot for super::Linkbot {
    fn analog_write(&mut self, pin: u8, value: u8) -> Result<(), String>
    {
        self.write_twi(TWI_ADDR, 
                       vec![Command::Header as u8,
                            Command::AnalogWrite as u8,
                            pin,
                            value])
    }

    fn analog_read(&mut self, pin: u8) -> Result<u16, String> {
        let data = self.write_read_twi(TWI_ADDR,
                            2,
                            vec![Command::Header as u8,
                                 Command::AnalogRead as u8,
                                 pin])?;
        if data.len() != 2 {
            Err(format!("analog_read expected 2 bytes from TWI. Got {} bytes.", data.len()))
        } else {
            let value:u16 = (data[0] as u16)<<8 | data[1] as u16;
            Ok(value)
        }
    }

    fn digital_write(&mut self, pin: u8, value: u8) -> Result<(), String>
    {
        self.write_twi(TWI_ADDR, 
                       vec![Command::Header as u8,
                            Command::DigitalWrite as u8,
                            pin,
                            value])
    }

    fn digital_read(&mut self, pin: u8) -> Result<u8, String>
    {
        let data = self.write_read_twi(TWI_ADDR,
                            1,
                            vec![Command::Header as u8,
                                 Command::DigitalRead as u8,
                                 pin])?;
        if data.len() != 1 {
            Err(format!("analog_read expected 1 bytes from TWI. Got {} bytes.", data.len()))
        } else {
            Ok(data[0])
        }

    }

    fn set_pin_mode(&mut self, pin: u8, mode: PinMode) -> Result<(), String>
    {
        self.write_twi(TWI_ADDR, 
                       vec![Command::Header as u8,
                            Command::PinMode as u8,
                            pin,
                            mode as u8])
    }
}
