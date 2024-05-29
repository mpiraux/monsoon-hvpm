use std::{
    collections::HashMap,
    iter::Sum,
    ops::{Add, Div},
    sync::{atomic::AtomicBool, Arc},
    time::{Duration, SystemTime},
};

use clap::ValueEnum;
use log::{trace, warn};
use rusb::{Device, DeviceHandle, GlobalContext};

const VENDOR_ID: u16 = 0x2ab9;
const PRODUCT_ID: u16 = 0x0001;
const FLOAT_TO_INT_PIC: f64 = 1048576.0;

const SUPPORTED_HW_MODEL: u32 = 2;
const SUPPORTED_FW_VERSION: u32 = 32;

const USB_TIMEOUT: Duration = Duration::from_secs(5);
const CAL_TIME: u16 = 1250;

const FINE_THRESHOLD: u16 = 64000;
const AUX_FINE_THRESHOLD: u16 = 30000;
const ADC_RATIO: f64 = 6.25e-05;
const MAIN_VOLTAGE_SCALE: f64 = 4.0;
const USB_VOLTAGE_SCALE: f64 = 2.0;

pub const SAMPLECOUNT_INFINITE: u32 = 0xFFFFFFFF;

#[derive(Clone, Copy)]
enum EepromOpCode {
    FirstOpCode = 0x02,
    UsbPassthroughMode = 0x10, // Sets USB Passthrough mode according to value.  Off = 0, On = 1, Auto = 2
    MainFineScale = 0x1a,      // HVPM Calibration value, 32-bits, unsigned
    MainCoarseScale = 0x1b,    // HVPM Calibration value, 32-bits, unsigned
    USBFineScale = 0x1c,       // HVPM Calibration value, 32-bits, unsigned
    USBCoarseScale = 0x1d,     // HVPM Calibration value, 32-bits, unsigned
    AuxFineScale = 0x1e,       // HVPM Calibration value, 32-bits, unsigned
    AuxCoarseScale = 0x1f,     // HVPM Calibration value, 32-bits, unsigned
    SetVoltageChannel = 0x23, // Sets voltage channel: Value 00 = Main & USB voltage measurements. Value 01 = Main & Aux voltage measurements
    MainFineZeroOffset = 0x25, // Zero-level offset
    MainCoarseZeroOffset = 0x26, // Zero-level offset
    USBFineZeroOffset = 0x27, // Zero-level offset
    USBCoarseZeroOffset = 0x28, // Zero-level offset
    SetRunCurrentLimit = 0x44, // Sets runtime current limit HV Amps = 15.625*(1.0-powerupCurrentLimit/65535)
    SetMainVoltage = 0x41,     // Voltage = value * 1048576
    GetSerialNumber = 0x42,
    GetFirmwareVersion = 0xc0, // Read-only, gets the firmware version
    GetProtocolVersion = 0xc1, // Read-only, gets the Protocol version
    GetHardwareModel = 0x45,   // 0 = unknown, 1 = LV, 2 = HV
    GetStartStatus = 0xc4,
    GetDacCalLow = 0x88,  // 2.5V ADC Reference Calibration
    GetDacCalHigh = 0x89, // 4.096V ADC Reference Calibration
    Stop = 0xff,
}

#[derive(Clone, Copy)]
enum UsbControlCodes {
    InPacket = 0xc0,
    OutPacket = 0x40,
    RequestStart = 0x02,
    RequestStop = 0x03,
    SetValue = 0x01,
    RequestResetToBootloader = 0xff,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
enum CalibrationType {
    Coarse,
    Fine,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
enum CalibrationKind {
    Reference,
    Zero,
}

const NO_CAL: usize = 5;
struct Calibration {
    values: [u16; NO_CAL],
    idx: usize,
    calibrated: bool,
}

impl Calibration {
    fn new() -> Calibration {
        Calibration {
            values: [0; NO_CAL],
            idx: 0,
            calibrated: false,
        }
    }
    #[inline]
    fn get(&self) -> u16 {
        assert!(self.calibrated);
        (self.values.iter().copied().fold(0u32, |f, v| f + v as u32) / self.values.len() as u32)
            as u16
    }
    #[inline]
    fn add(&mut self, v: u16) {
        self.values[self.idx] = v;
        self.idx = (self.idx + 1) % self.values.len();
        if self.idx == 0 {
            self.calibrated = true;
        }
    }
}

struct ChannelCalibration {
    data: HashMap<(CalibrationKind, CalibrationType), Calibration>,
}

impl ChannelCalibration {
    fn new() -> ChannelCalibration {
        let mut data = HashMap::new();
        data.insert(
            (CalibrationKind::Reference, CalibrationType::Fine),
            Calibration::new(),
        );
        data.insert(
            (CalibrationKind::Reference, CalibrationType::Coarse),
            Calibration::new(),
        );
        data.insert(
            (CalibrationKind::Zero, CalibrationType::Fine),
            Calibration::new(),
        );
        data.insert(
            (CalibrationKind::Zero, CalibrationType::Coarse),
            Calibration::new(),
        );
        ChannelCalibration { data }
    }

    #[inline]
    fn calibrated(&self) -> bool {
        self.data.values().all(|v| v.calibrated)
    }

    #[inline]
    fn get(&self, cal_kind: CalibrationKind, cal_type: CalibrationType) -> u16 {
        self.data.get(&(cal_kind, cal_type)).unwrap().get()
    }

    #[inline]
    fn add(&mut self, cal_kind: CalibrationKind, cal_type: CalibrationType, v: u16) {
        assert_ne!(v, 0);
        self.data.get_mut(&(cal_kind, cal_type)).unwrap().add(v);
    }
}

#[derive(Clone, Copy, Debug)]
enum SampleType {
    Measurement = 0x00,
    ZeroCalibration = 0x10,
    Invalid = 0x20,
    RefCal = 0x30,
}

impl TryFrom<u8> for SampleType {
    type Error = String;

    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            x if x == SampleType::Measurement as u8 => Ok(SampleType::Measurement),
            x if x == SampleType::ZeroCalibration as u8 => Ok(SampleType::ZeroCalibration),
            x if x == SampleType::Invalid as u8 => Ok(SampleType::Invalid),
            x if x == SampleType::RefCal as u8 => Ok(SampleType::RefCal),
            _ => Err(format!("Unknown sample type {v}")),
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct HardwareSample {
    values: [u16; 8],
    sample_type: SampleType,
    unused: u8,
    timestamp: SystemTime,
}

#[derive(Clone, Copy, Debug)]
pub struct SoftwareSample {
    pub main_current: f64,
    pub usb_current: f64,
    pub aux_current: f64,
    pub main_voltage: f64,
    pub usb_voltage: f64,
    pub timestamp: SystemTime,
}

impl Default for SoftwareSample {
    fn default() -> Self {
        Self::new()
    }
}

impl SoftwareSample {
    pub fn new() -> SoftwareSample {
        SoftwareSample {
            main_current: 0.,
            usb_current: 0.,
            aux_current: 0.,
            main_voltage: 0.,
            usb_voltage: 0.,
            timestamp: SystemTime::now(),
        }
    }

    pub fn csv_header() -> &'static str {
        "timestamp,main_voltage,usb_voltage,main_current,usb_current,aux_current"
    }

    pub fn csv_row(&self) -> String {
        format!(
            "{},{},{},{},{},{}",
            self.timestamp
                .duration_since(SystemTime::UNIX_EPOCH)
                .unwrap()
                .as_micros(),
            self.main_voltage,
            self.usb_voltage,
            self.main_current,
            self.usb_current,
            self.aux_current
        )
    }
}

impl Add<&SoftwareSample> for SoftwareSample {
    type Output = SoftwareSample;

    fn add(self, rhs: &SoftwareSample) -> Self::Output {
        SoftwareSample {
            main_current: self.main_current + rhs.main_current,
            usb_current: self.usb_current + rhs.usb_current,
            aux_current: self.aux_current + rhs.aux_current,
            main_voltage: self.main_voltage + rhs.main_voltage,
            usb_voltage: self.usb_voltage + rhs.usb_voltage,
            timestamp: self.timestamp.min(rhs.timestamp),
        }
    }
}

impl Div<f64> for SoftwareSample {
    type Output = SoftwareSample;

    fn div(self, rhs: f64) -> Self::Output {
        SoftwareSample {
            main_current: self.main_current / rhs,
            usb_current: self.usb_current / rhs,
            aux_current: self.aux_current / rhs,
            main_voltage: self.main_voltage / rhs,
            usb_voltage: self.usb_voltage / rhs,
            timestamp: self.timestamp,
        }
    }
}

impl<'a> Sum<&'a SoftwareSample> for SoftwareSample {
    fn sum<I: Iterator<Item = &'a SoftwareSample>>(iter: I) -> Self {
        iter.fold(SoftwareSample::new(), |a, s| a + s)
    }
}

#[derive(Default)]
struct Status {
    firmware_version: u16,
    protocol_version: u8,
    serial_number: u16,
    hardware_model: u16,

    power_up_current_limit: u16,
    runtime_current_limit: u16,
    power_up_time: u8,
    temperature_limit: u16,
    usb_passthrough_mode: u8,

    dac_cal_high: u16,
    dac_cal_low: u16,

    main_fine_scale: u16,
    main_coarse_scale: u16,
    main_fine_zero_offset: i16,
    main_coarse_zero_offset: i16,

    usb_fine_scale: u16,
    usb_coarse_scale: u16,
    usb_fine_zero_offset: i16,
    usb_coarse_zero_offset: i16,

    aux_fine_scale: u16,
    aux_coarse_scale: u16,
}

impl Status {
    fn fill(&mut self, device: &DeviceHandle<GlobalContext>) -> Result<(), rusb::Error> {
        self.firmware_version =
            read_usb_control_value(device, EepromOpCode::GetFirmwareVersion)? as u16;
        self.protocol_version =
            read_usb_control_value(device, EepromOpCode::GetProtocolVersion)? as u8;
        self.serial_number =
            read_usb_control_value(device, EepromOpCode::GetFirmwareVersion)? as u16;
        self.hardware_model =
            read_usb_control_value(device, EepromOpCode::GetHardwareModel)? as u16;
        // TODO(mp): Read current limit, power up time
        self.usb_passthrough_mode =
            read_usb_control_value(device, EepromOpCode::UsbPassthroughMode)? as u8;
        self.dac_cal_high = read_usb_control_value(device, EepromOpCode::GetDacCalHigh)? as u16;
        self.dac_cal_low = read_usb_control_value(device, EepromOpCode::GetDacCalLow)? as u16;

        self.main_fine_scale = read_usb_control_value(device, EepromOpCode::MainFineScale)? as u16;
        self.main_coarse_scale =
            read_usb_control_value(device, EepromOpCode::MainCoarseScale)? as u16;
        self.main_fine_zero_offset =
            read_usb_control_value(device, EepromOpCode::MainFineZeroOffset)? as i16;
        self.main_coarse_zero_offset =
            read_usb_control_value(device, EepromOpCode::MainCoarseZeroOffset)? as i16;

        self.usb_fine_scale = read_usb_control_value(device, EepromOpCode::USBFineScale)? as u16;
        self.usb_coarse_scale =
            read_usb_control_value(device, EepromOpCode::USBCoarseScale)? as u16;
        self.usb_fine_zero_offset =
            read_usb_control_value(device, EepromOpCode::USBFineZeroOffset)? as i16;
        self.usb_coarse_zero_offset =
            read_usb_control_value(device, EepromOpCode::USBCoarseZeroOffset)? as i16;

        self.aux_fine_scale = read_usb_control_value(device, EepromOpCode::AuxFineScale)? as u16;
        self.aux_coarse_scale =
            read_usb_control_value(device, EepromOpCode::AuxCoarseScale)? as u16;

        Ok(())
    }
}

#[derive(Clone, Copy, ValueEnum)]
pub enum UsbPassthroughMode {
    Off = 0,
    On = 1,
    Auto = 2,
}

fn find_usb_device() -> Option<Device<GlobalContext>> {
    for device in rusb::devices().unwrap().iter() {
        let device_desc = device.device_descriptor().unwrap();
        if device_desc.vendor_id() == VENDOR_ID && device_desc.product_id() == PRODUCT_ID {
            return Some(device);
        }
    }
    None
}

fn is_ready_for(
    device: &DeviceHandle<GlobalContext>,
    opcode: EepromOpCode,
) -> Result<bool, rusb::Error> {
    let status = read_usb_control_value(device, opcode)?;
    Ok(!(0x80 & (status & 0xff)) == 0x80)
}

fn write_stop(device: &DeviceHandle<GlobalContext>) -> Result<(), rusb::Error> {
    device.write_control(
        UsbControlCodes::OutPacket as u8,
        UsbControlCodes::RequestStop as u8,
        0,
        0,
        &[],
        USB_TIMEOUT,
    )?;
    Ok(())
}

fn read_usb_control_value(
    device: &DeviceHandle<GlobalContext>,
    opcode: EepromOpCode,
) -> Result<u32, rusb::Error> {
    let mut buf = [0; 4];
    let read = device.read_control(
        UsbControlCodes::InPacket as u8,
        UsbControlCodes::SetValue as u8,
        0,
        opcode as u16,
        &mut buf,
        USB_TIMEOUT,
    )?;
    assert_eq!(read, 4);
    Ok(u32::from_le_bytes(buf))
}

fn write_usb_control_value(
    device: &DeviceHandle<GlobalContext>,
    opcode: EepromOpCode,
    value: u32,
) -> Result<(), rusb::Error> {
    if !is_ready_for(device, opcode)? {
        write_stop(device)?;
    }
    let value_array = value.to_le_bytes();
    let operation = opcode as u8;
    let value = u16::from_le_bytes([value_array[0], value_array[1]]);
    let index = u16::from_le_bytes([operation, value_array[2]]);
    trace!("write_control(OUT_PACKET, SET_VALUE, {value}, {index}, value_array, ..)");
    let written = device.write_control(
        UsbControlCodes::OutPacket as u8,
        UsbControlCodes::SetValue as u8,
        value,
        index,
        &value_array,
        USB_TIMEOUT,
    )?;
    assert_eq!(written, 4);
    Ok(())
}

fn find_and_connect() -> Result<Device<GlobalContext>, rusb::Error> {
    let device = find_usb_device().ok_or(rusb::Error::NoDevice)?;
    let mut handle = open_device(&device)?;
    let hw_model = read_usb_control_value(&handle, EepromOpCode::GetHardwareModel)?;
    let fw_version = read_usb_control_value(&handle, EepromOpCode::GetFirmwareVersion)?;

    if hw_model != SUPPORTED_HW_MODEL || fw_version != SUPPORTED_FW_VERSION {
        warn!("Found unsupported device hw={hw_model}, fw={fw_version}");
        return Err(rusb::Error::NoDevice);
    }
    let _detach_result = handle.detach_kernel_driver(0); // This supposedly detach the USB HID driver

    let _config = device.config_descriptor(handle.active_configuration()?);
    Ok(device)
}

fn open_device(device: &Device<GlobalContext>) -> Result<DeviceHandle<GlobalContext>, rusb::Error> {
    let mut handle = device.open()?;
    if let Ok(active) = handle.kernel_driver_active(0) {
        if active {
            handle.detach_kernel_driver(0)?;
        }
    }
    handle.claim_interface(0)?;
    Ok(handle)
}

pub struct HVPM {
    device: Device<GlobalContext>,
    calibration: [ChannelCalibration; 3],
    status: Status,
    samples: Vec<SoftwareSample>,
    dropped: usize,
}

impl HVPM {
    pub fn new() -> Result<Self, rusb::Error> {
        let mut d = HVPM {
            device: find_and_connect()?,
            calibration: [
                ChannelCalibration::new(),
                ChannelCalibration::new(),
                ChannelCalibration::new(),
            ],
            status: Status {
                ..Default::default()
            },
            samples: Vec::with_capacity(1000),
            dropped: 0,
        };
        d.status.fill(&open_device(&d.device)?)?;
        Ok(d)
    }

    /// Sets the output voltage. A positive voltage enables the output. A zero voltage disables it.
    pub fn set_vout(&self, voltage: f64) -> Result<(), rusb::Error> {
        assert!(voltage >= 0.);
        let handle = open_device(&self.device)?;
        let dac_calc_high = read_usb_control_value(&handle, EepromOpCode::GetDacCalHigh)?;
        let dac_calc_low = read_usb_control_value(&handle, EepromOpCode::GetDacCalLow)?;

        if dac_calc_low <= 0xd000 || dac_calc_low >= 0xf000 {
            warn!("Found DAC calc out of tolerance");
            return Err(rusb::Error::NoDevice);
        }
        if dac_calc_high <= 0xc000 || dac_calc_high >= 0xd000 {
            warn!("Found DAC calc out of tolerance");
            return Err(rusb::Error::NoDevice);
        }

        let voltage_int = (voltage * FLOAT_TO_INT_PIC) as u32;
        write_usb_control_value(&handle, EepromOpCode::SetMainVoltage, voltage_int)?;
        Ok(())
    }

    /// Starts the sampling for the given duration. When no duration is given, starts it for an indefinite period.
    pub fn start_sampling(&self, duration: Option<Duration>) -> Result<(), rusb::Error> {
        let handle = open_device(&self.device)?;
        if !is_ready_for(&handle, EepromOpCode::FirstOpCode)? {
            write_stop(&handle)?;
        }
        let max_time = duration
            .map(|d| (d.as_millis() * 5) as u32)
            .unwrap_or(SAMPLECOUNT_INFINITE);
        let max_time_bytes = max_time.to_le_bytes();
        let value = CAL_TIME.to_le();
        let index = 0;
        trace!("write_control(OUT_PACKET, REQUEST_START, {value}, {index}, {max_time}, ..)");
        let written = handle.write_control(
            UsbControlCodes::OutPacket as u8,
            UsbControlCodes::RequestStart as u8,
            value,
            index,
            &max_time_bytes,
            USB_TIMEOUT,
        )?;
        assert_eq!(written, 4);
        Ok(())
    }

    /// Stops the sampling.
    pub fn stop_sampling(&self) -> Result<(), rusb::Error> {
        write_stop(&open_device(&self.device)?)?;
        Ok(())
    }

    fn process_sample(&mut self, sample: HardwareSample) {
        match sample.sample_type {
            SampleType::Measurement => {
                if !self.calibration.iter().all(|c| c.calibrated()) {
                    warn!("Skipping measurement because of lack of calibration");
                    return;
                }

                let main_current = {
                    let zero_offset = self.calibration[0]
                        .get(CalibrationKind::Zero, CalibrationType::Coarse)
                        as i32
                        + self.status.main_coarse_zero_offset as i32;
                    let cal_ref = self.calibration[0]
                        .get(CalibrationKind::Reference, CalibrationType::Coarse);
                    let slope = if cal_ref as i32 - zero_offset != 0 {
                        self.status.main_coarse_scale as f64 / (cal_ref as f64 - zero_offset as f64)
                    } else {
                        0.
                    };
                    let main_coarse_current =
                        (sample.values[0] as i32 - zero_offset) as f64 * slope;

                    let zero_offset = self.calibration[0]
                        .get(CalibrationKind::Zero, CalibrationType::Fine)
                        as i32
                        + self.status.main_fine_zero_offset as i32;
                    let cal_ref =
                        self.calibration[0].get(CalibrationKind::Reference, CalibrationType::Fine);
                    let slope = if cal_ref as i32 - zero_offset != 0 {
                        self.status.main_fine_scale as f64 / (cal_ref as f64 - zero_offset as f64)
                    } else {
                        0.
                    };
                    let main_fine_current =
                        (sample.values[1] as i32 - zero_offset) as f64 * slope / 1000.;

                    if sample.values[1] < FINE_THRESHOLD {
                        main_fine_current
                    } else {
                        main_coarse_current
                    }
                };

                let usb_current = {
                    let zero_offset = self.calibration[1]
                        .get(CalibrationKind::Zero, CalibrationType::Coarse)
                        as i32
                        + self.status.usb_coarse_zero_offset as i32;
                    let cal_ref = self.calibration[1]
                        .get(CalibrationKind::Reference, CalibrationType::Coarse);
                    let slope = if cal_ref as i32 - zero_offset != 0 {
                        self.status.usb_coarse_scale as f64 / (cal_ref as f64 - zero_offset as f64)
                    } else {
                        0.
                    };
                    let usb_coarse_current = (sample.values[2] as i32 - zero_offset) as f64 * slope;

                    let zero_offset = self.calibration[1]
                        .get(CalibrationKind::Zero, CalibrationType::Fine)
                        as i32
                        + self.status.usb_fine_zero_offset as i32;
                    let cal_ref =
                        self.calibration[1].get(CalibrationKind::Reference, CalibrationType::Fine);
                    let slope = if cal_ref as i32 - zero_offset != 0 {
                        self.status.usb_fine_scale as f64 / (cal_ref as f64 - zero_offset as f64)
                    } else {
                        0.
                    };
                    let usb_fine_current =
                        (sample.values[3] as i32 - zero_offset) as f64 * slope / 1000.;

                    if sample.values[3] < FINE_THRESHOLD {
                        usb_fine_current
                    } else {
                        usb_coarse_current
                    }
                };

                let aux_current = {
                    let zero_offset = self.calibration[2]
                        .get(CalibrationKind::Zero, CalibrationType::Coarse)
                        as i32;
                    let cal_ref = self.calibration[2]
                        .get(CalibrationKind::Reference, CalibrationType::Coarse);
                    let slope = if cal_ref as i32 - zero_offset != 0 {
                        self.status.aux_coarse_scale as f64 / (cal_ref as f64 - zero_offset as f64)
                    } else {
                        0.
                    };
                    let aux_coarse_current = (sample.values[4] as i32 - zero_offset) as f64 * slope;

                    let zero_offset = self.calibration[2]
                        .get(CalibrationKind::Zero, CalibrationType::Fine)
                        as i32;
                    let cal_ref =
                        self.calibration[2].get(CalibrationKind::Reference, CalibrationType::Fine);
                    let slope = if cal_ref as i32 - zero_offset != 0 {
                        self.status.aux_fine_scale as f64 / (cal_ref as f64 - zero_offset as f64)
                    } else {
                        0.
                    };
                    let aux_fine_current =
                        (sample.values[5] as i32 - zero_offset) as f64 * slope / 1000.;

                    if sample.values[5] < AUX_FINE_THRESHOLD {
                        aux_fine_current
                    } else {
                        aux_coarse_current
                    }
                };

                let main_voltage = sample.values[6] as f64 * ADC_RATIO * MAIN_VOLTAGE_SCALE;
                let usb_voltage = sample.values[7] as f64 * ADC_RATIO * USB_VOLTAGE_SCALE;

                self.samples.push(SoftwareSample {
                    main_current,
                    usb_current,
                    aux_current,
                    main_voltage,
                    usb_voltage,
                    timestamp: sample.timestamp,
                });

                trace!("Main current {main_current}, USB current {usb_current}, Aux current {aux_current}, Main voltage {main_voltage}, USB voltage {usb_voltage}");
            }
            SampleType::ZeroCalibration => {
                let cal_kind = CalibrationKind::Zero;
                for i in 0..6 {
                    self.calibration[i / 2].add(
                        cal_kind,
                        if i % 2 == 0 {
                            CalibrationType::Coarse
                        } else {
                            CalibrationType::Fine
                        },
                        sample.values[i],
                    );
                }
            }
            SampleType::Invalid => {}
            SampleType::RefCal => {
                let cal_kind = CalibrationKind::Reference;
                for i in 0..6 {
                    self.calibration[i / 2].add(
                        cal_kind,
                        if i % 2 == 0 {
                            CalibrationType::Coarse
                        } else {
                            CalibrationType::Fine
                        },
                        sample.values[i],
                    );
                }
            }
        }
    }

    fn read_samples(&mut self, device: &DeviceHandle<GlobalContext>) -> Result<(), rusb::Error> {
        let mut packet = [0; 64];
        let read = device.read_bulk(0x81, &mut packet, Duration::from_millis(250))?;
        for sample in self.decode_packet(&packet[..read]).into_iter().flatten() {
            self.process_sample(sample);
        }
        Ok(())
    }

    fn decode_packet(&mut self, packet: &[u8]) -> [Option<HardwareSample>; 3] {
        let now = SystemTime::now();
        let dropped = u16::from_le_bytes([packet[0], packet[1]]);
        self.dropped = dropped as usize;
        let _flags = packet[2];
        let no_samples = packet[3] as usize;

        let mut samples = [None; 3];
        let mut packet_iter = packet.iter().skip(4);
        for sample in samples.iter_mut().take(no_samples) {
            let mut values = [0; 8];
            for v in &mut values {
                *v = u16::from_be_bytes([
                    *packet_iter.next().unwrap(),
                    *packet_iter.next().unwrap(),
                ]);
            }

            *sample = Some(HardwareSample {
                values,
                unused: *packet_iter.next().unwrap(),
                sample_type: (*packet_iter.next().unwrap() & 0x30).try_into().unwrap(),
                timestamp: now,
            });
            trace!("Read sample {:?}", sample.unwrap());
        }

        samples
    }

    /// Captures available samples until the given stop condition becomes true.
    pub fn capture_samples_until(&mut self, stop: Arc<AtomicBool>) -> Result<(), rusb::Error> {
        let device = open_device(&self.device)?;
        while !stop.load(std::sync::atomic::Ordering::SeqCst) {
            match self.read_samples(&device) {
                Ok(()) => {}
                Err(rusb::Error::Timeout) => break,
                Err(e) => return Err(e),
            }
        }
        Ok(())
    }

    /// Captures available samples until the end of the sampling period configured in [`HVPM::start_sampling()`].
    pub fn capture_samples(&mut self) -> Result<(), rusb::Error> {
        let device = open_device(&self.device)?;
        loop {
            match self.read_samples(&device) {
                Ok(()) => {}
                Err(rusb::Error::Timeout) => break,
                Err(e) => return Err(e),
            }
        }
        Ok(())
    }

    /// Moves out the captured samples and dropped counter.
    pub fn captured_samples(&mut self) -> (Vec<SoftwareSample>, usize) {
        let ret = (self.samples.drain(..).collect(), self.dropped);
        self.dropped = 0;
        ret
    }

    /// Sets the USB passthrough mode
    pub fn set_usb_passthrough(&self, mode: UsbPassthroughMode) -> Result<(), rusb::Error> {
        write_usb_control_value(
            &open_device(&self.device)?,
            EepromOpCode::UsbPassthroughMode,
            mode as u32,
        )?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use crate::*;
    use std::time::{Duration, SystemTime};
    use test_log::test;

    #[test]
    fn sample_averages() {
        let samples = [
            SoftwareSample {
                main_current: 1.,
                usb_current: 2.,
                aux_current: 3.,
                main_voltage: 4.,
                usb_voltage: 5.,
                timestamp: SystemTime::now(),
            },
            SoftwareSample {
                main_current: 2.,
                usb_current: 3.,
                aux_current: 4.,
                main_voltage: 5.,
                usb_voltage: 6.,
                timestamp: SystemTime::now(),
            },
        ];
        let average = samples.iter().sum::<SoftwareSample>() / samples.len() as f64;
        assert!(average.main_current == 1.5);
        assert!(average.usb_current == 2.5);
        assert!(average.aux_current == 3.5);
        assert!(average.main_voltage == 4.5);
        assert!(average.usb_voltage == 5.5);
    }

    #[test]
    fn test_bulk_read() {
        let mut device = HVPM::new().unwrap();
        device.stop_sampling().unwrap();
        device.set_vout(1.).unwrap();
        device.start_sampling(Some(Duration::from_secs(5))).unwrap();
        device.capture_samples().unwrap();
        device.set_vout(0.).unwrap();
        let (collected, dropped) = device.captured_samples();
        println!("collected: {}, dropped: {}", collected.len(), dropped);
    }
}
