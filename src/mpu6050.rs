use bitfield_struct::bitfield;
use embedded_hal::{delay::DelayNs, i2c::I2c};

use mpu6050 as _;

/// MPU6050 Data Frame from register 0x3B to 0x48
pub struct Mpu6050DataFrame {
    pub accel_x: i16,
    pub accel_y: i16,
    pub accel_z: i16,
    pub temp: i16,
    pub gyro_x: i16,
    pub gyro_y: i16,
    pub gyro_z: i16,
}

// implement from be_bytes for Mpu6050DataFrame
impl From<[u8; 14]> for Mpu6050DataFrame {
    fn from(buf: [u8; 14]) -> Self {
        Mpu6050DataFrame {
            accel_x: i16::from_be_bytes([buf[0], buf[1]]),
            accel_y: i16::from_be_bytes([buf[2], buf[3]]),
            accel_z: i16::from_be_bytes([buf[4], buf[5]]),
            temp: i16::from_be_bytes([buf[6], buf[7]]),
            gyro_x: i16::from_be_bytes([buf[8], buf[9]]),
            gyro_y: i16::from_be_bytes([buf[10], buf[11]]),
            gyro_z: i16::from_be_bytes([buf[12], buf[13]]),
        }
    }
}

// impl Mpu6050DataFrame {
//     /// pitch in degress, scaled by 1000
//     pub fn calculate_pitch(&self) -> i32 {
//         let acc_x = self.accel_x as f32;
//         let acc_z = self.accel_z as f32;

//         (-fast_math::atan2(acc_x, acc_z) * 180.0 * 1000.0 / 3.14159) as i32
//     }
// }

pub trait ImuRead {
    type Error;
    // add fetching functions from below
    fn get_data_frame(&mut self) -> Result<Mpu6050DataFrame, Self::Error>;
    fn get_accel_raw(&mut self) -> Result<(i16, i16, i16), Self::Error>;
    fn get_accel(&mut self) -> Result<(f32, f32, f32), Self::Error>;
    fn get_gyro_raw(&mut self) -> Result<(i16, i16, i16), Self::Error>;
    fn get_gyro(&mut self) -> Result<(f32, f32, f32), Self::Error>;
}

pub struct Mpu6050<I> {
    i2c: I,
    addr: u8,
    acc_sensitivity: f32,
    gyro_sensitivity: f32,
}

impl<I, E> Mpu6050<I>
where
    I: I2c<Error = E>,
{
    const ADDR: u8 = 0x68;

    pub fn new(i2c: I) -> Self {
        Mpu6050 {
            i2c,
            addr: Self::ADDR,
            acc_sensitivity: 16384.0,
            gyro_sensitivity: 131.0,
        }
    }

    pub fn init<D>(&mut self, delay: &mut D) -> Result<(), Mpu6050Error<E>>
    where
        D: DelayNs,
    {
        // wake up
        self.write_byte(PwrMgmt1::ADDR, PwrMgmt1::default())?;
        delay.delay_ms(100);

        // check who am i
        let who_am_i = self.read_byte(WhoAmI::ADDR)?;
        if who_am_i != WhoAmI::VALUE {
            return Err(Mpu6050Error::IncorrectWhoAmI(who_am_i));
        }

        // set accel and gyro range
        self.write_byte(AccelConfig::ADDR, AccelConfig::default())?;
        self.write_byte(GyroConfig::ADDR, GyroConfig::default())?;

        // set DLPF_CFG
        self.set_dlpf_cfg(DlpfCfg::Mode0)?;

        // enable motion interrupt
        self.write_byte(IntEnable::ADDR, IntEnable::new().with_data_rdy_en(true))?;

        // set motion threshold
        self.write_byte(MotThr::ADDR, MotThr::new().with_mot_thr(0))?;

        self.write_byte(
            IntPinCfg::ADDR,
            IntPinCfg::new()
                .with_int_level(true)
                .with_latch_int_en(true), // .with_int_open(true)
                                          // .with_int_rd_clear(true),
        )?;

        Ok(())
    }

    pub fn set_dlpf_cfg(&mut self, mode: DlpfCfg) -> Result<(), Mpu6050Error<E>> {
        self.write_byte(Config::ADDR, Config::new().with_dlpf_cfg(mode))
    }

    fn read_bytes(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Mpu6050Error<E>> {
        Ok(self.i2c.write_read(self.addr, &[reg], buf)?)
    }

    fn read_byte(&mut self, reg: u8) -> Result<u8, Mpu6050Error<E>> {
        let mut buf = [0];
        self.i2c.write_read(self.addr, &[reg], &mut buf)?;

        Ok(buf[0])
    }

    fn write_byte(&mut self, reg: u8, byte: impl Into<u8>) -> Result<(), Mpu6050Error<E>> {
        Ok(self.i2c.write(self.addr, &[reg, byte.into()])?)
    }
}

impl<I, E> ImuRead for Mpu6050<I>
where
    I: I2c<Error = E>,
{
    type Error = Mpu6050Error<E>;
    /// fetch accel, temp and gyro in a single transaction
    fn get_data_frame(&mut self) -> Result<Mpu6050DataFrame, Self::Error> {
        let mut buf = [0; 14];
        self.read_bytes(0x3B, &mut buf)?;

        Ok(Mpu6050DataFrame::from(buf))
    }

    /// not scaled by sensitivity (default 16384)
    fn get_accel_raw(&mut self) -> Result<(i16, i16, i16), Self::Error> {
        let mut buf = [0; 6];
        self.read_bytes(0x3B, &mut buf)?;
        Ok((
            i16::from_be_bytes([buf[0], buf[1]]),
            i16::from_be_bytes([buf[2], buf[3]]),
            i16::from_be_bytes([buf[4], buf[5]]),
        ))
    }

    /// in g
    fn get_accel(&mut self) -> Result<(f32, f32, f32), Self::Error> {
        let (x, y, z) = self.get_accel_raw()?;
        Ok((
            x as f32 / self.acc_sensitivity,
            y as f32 / self.acc_sensitivity,
            z as f32 / self.acc_sensitivity,
        ))
    }

    /// not scaled by sensitivity (default 131)
    fn get_gyro_raw(&mut self) -> Result<(i16, i16, i16), Self::Error> {
        let mut buf = [0; 6];
        self.read_bytes(0x43, &mut buf)?;
        Ok((
            i16::from_be_bytes([buf[0], buf[1]]),
            i16::from_be_bytes([buf[2], buf[3]]),
            i16::from_be_bytes([buf[4], buf[5]]),
        ))
    }

    fn get_gyro(&mut self) -> Result<(f32, f32, f32), Self::Error> {
        let (x, y, z) = self.get_gyro_raw()?;
        Ok((
            x as f32 / self.gyro_sensitivity,
            y as f32 / self.gyro_sensitivity,
            z as f32 / self.gyro_sensitivity,
        ))
    }
}

#[derive(thiserror_no_std::Error, Debug)]
pub enum Mpu6050Error<E> {
    #[error("I2C error: {0}")]
    I2cError(#[from] E),
    #[error("Incorrect WHO_AM_I: 0x{0:02X}")]
    IncorrectWhoAmI(u8),
}

#[bitfield(u8)]
pub struct PwrMgmt1 {
    #[bits(3, default=CLKSEL::PlLwithXAxisGyro)]
    pub clksel: CLKSEL,
    pub temp_dis: bool,
    __: bool,
    pub cycle: bool,
    pub sleep: bool,
    pub device_reset: bool,
}

impl PwrMgmt1 {
    pub const ADDR: u8 = 0x6B;
}

#[derive(Debug)]
pub enum CLKSEL {
    Internal8MHz = 0,
    PlLwithXAxisGyro = 1,
    PlLwithYAxisGyro = 2,
    PlLwithZAxisGyro = 3,
    // ...
    Stop = 7,
}
impl CLKSEL {
    const fn into_bits(self) -> u8 {
        self as _
    }

    const fn from_bits(bits: u8) -> Self {
        match bits {
            0 => Self::Internal8MHz,
            1 => Self::PlLwithXAxisGyro,
            2 => Self::PlLwithYAxisGyro,
            3 => Self::PlLwithZAxisGyro,
            // ...
            7 => Self::Stop,
            _ => unimplemented!(),
        }
    }
}

pub struct WhoAmI;

impl WhoAmI {
    pub const ADDR: u8 = 0x75;
    pub const VALUE: u8 = 0x68;
}

#[bitfield(u8)]
pub struct GyroConfig {
    #[bits(3)]
    __: u8,
    #[bits(2, default=FsSel::PlusMinus250)]
    pub fs_sel: FsSel,
    pub zg_st: bool,
    pub yg_st: bool,
    pub xg_st: bool,
}

impl GyroConfig {
    pub const ADDR: u8 = 0x1B;
}

#[derive(Debug)]
pub enum FsSel {
    PlusMinus250 = 0,
    PlusMinus500 = 1,
    PlusMinus1000 = 2,
    PlusMinus2000 = 3,
}

impl FsSel {
    const fn into_bits(self) -> u8 {
        self as _
    }

    const fn from_bits(bits: u8) -> Self {
        match bits {
            0 => Self::PlusMinus250,
            1 => Self::PlusMinus500,
            2 => Self::PlusMinus1000,
            3 => Self::PlusMinus2000,
            _ => unimplemented!(),
        }
    }
}

#[bitfield(u8)]
pub struct AccelConfig {
    #[bits(3)]
    __: u8,
    #[bits(2, default=AfsSel::PlusMinus2G)]
    afs_sel: AfsSel,
    za_st: bool,
    ya_st: bool,
    xa_st: bool,
}

impl AccelConfig {
    pub const ADDR: u8 = 0x1C;
}

#[derive(Debug)]
enum AfsSel {
    PlusMinus2G = 0,
    PlusMinus4G = 1,
    PlusMinus8G = 2,
    PlusMinus16G = 3,
}

impl AfsSel {
    const fn into_bits(self) -> u8 {
        self as _
    }

    const fn from_bits(bits: u8) -> Self {
        match bits {
            0 => Self::PlusMinus2G,
            1 => Self::PlusMinus4G,
            2 => Self::PlusMinus8G,
            3 => Self::PlusMinus16G,
            _ => unimplemented!(),
        }
    }
}

#[bitfield(u8)]
pub struct Config {
    #[bits(3, default=DlpfCfg::Mode0)]
    pub dlpf_cfg: DlpfCfg,
    #[bits(3, default=ExtSyncSet::Disabled)]
    pub ext_sync_set: ExtSyncSet,
    #[bits(2)]
    __: u8,
}

impl Config {
    pub const ADDR: u8 = 0x1A;
}

///  | Mode | Bandwidth(Hz) | Delay(ms) | Bandwidth(Hz) | Delay(ms) | Fs(kHz) |
///  |------| ------------- |---------- |-------------- |-----------|---------|
///  | 0    |     260       | 0         | 256           | 0.98      | 8       |
///  | 1    |     184       | 2.0       | 188           | 1.9       | 1       |
///  | 2    |     94        | 3.0       | 98            | 2.8       | 1       |
///  | 3    |     44        | 4.9       | 42            | 4.8       | 1       |
///  | 4    |     21        | 8.5       | 20            | 8.3       | 1       |
///  | 5    |     10        | 13.8      | 10            | 13.4      | 1       |
///  | 6    |     5         | 19.0      | 5             | 18.6      | 1       |
///  | 7    |     RESERVED  | RESERVED  | RESERVED      | RESERVED  | 8       |
#[derive(Debug)]
pub enum DlpfCfg {
    /// Acc: 260Hz, Gyro: 256Hz, Fs: 8kHz
    Mode0 = 0,
    /// Acc: 184Hz, Gyro: 188Hz, Fs: 1kHz
    Mode1 = 1,
    /// Acc: 94Hz, Gyro: 98Hz, Fs: 1kHz
    Mode2 = 2,
    /// Acc: 44Hz, Gyro: 42Hz, Fs: 1kHz
    Mode3 = 3,
    /// Acc: 21Hz, Gyro: 20Hz, Fs: 1kHz
    Mode4 = 4,
    /// Acc: 10Hz, Gyro: 10Hz, Fs: 1kHz
    Mode5 = 5,
    /// Acc: 5Hz, Gyro: 5Hz, Fs: 1kHz
    Mode6 = 6,
    /// Reserved, Fs: 8kHz
    Reserved = 7,
}

impl DlpfCfg {
    const fn into_bits(self) -> u8 {
        self as _
    }

    const fn from_bits(bits: u8) -> Self {
        match bits {
            0 => Self::Mode0,
            1 => Self::Mode1,
            2 => Self::Mode2,
            3 => Self::Mode3,
            4 => Self::Mode4,
            5 => Self::Mode5,
            6 => Self::Mode6,
            7 => Self::Reserved,
            _ => unimplemented!(),
        }
    }
}

#[derive(Debug)]
pub enum ExtSyncSet {
    Disabled = 0,
    TempOutL = 1,
    GyroXoutL = 2,
    GyroYoutL = 3,
    GyroZoutL = 4,
    AccelXoutL = 5,
    AccelYoutL = 6,
    AccelZoutL = 7,
}

impl ExtSyncSet {
    const fn into_bits(self) -> u8 {
        self as _
    }

    const fn from_bits(bits: u8) -> Self {
        match bits {
            0 => Self::Disabled,
            1 => Self::TempOutL,
            2 => Self::GyroXoutL,
            3 => Self::GyroYoutL,
            4 => Self::GyroZoutL,
            5 => Self::AccelXoutL,
            6 => Self::AccelYoutL,
            7 => Self::AccelZoutL,
            _ => unimplemented!(),
        }
    }
}

#[bitfield(u8)]
pub struct IntEnable {
    pub data_rdy_en: bool,
    pub __: bool,
    pub __: bool,
    pub i2c_mst_int_en: bool,
    pub fifo_oflow_en: bool,
    pub __: bool,
    pub mot_en: bool,
    pub __: bool,
}

impl IntEnable {
    pub const ADDR: u8 = 0x38;
}

#[bitfield(u8)]
pub struct IntPinCfg {
    __: bool,
    i2c_bypass_en: bool,
    fsync_int_en: bool,
    fsync_int_level: bool,
    int_rd_clear: bool,
    latch_int_en: bool,
    int_open: bool,
    int_level: bool,
}

impl IntPinCfg {
    pub const ADDR: u8 = 0x37;
}

// MOT_THR
#[bitfield(u8)]
pub struct MotThr {
    #[bits(7)]
    pub mot_thr: u8,
    __: bool,
}

impl MotThr {
    pub const ADDR: u8 = 0x1F;
}
