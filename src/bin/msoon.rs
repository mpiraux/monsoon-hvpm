use std::time::Duration;

use clap::Parser;
use itertools::Itertools;
use monsoon_hvpm::{SoftwareSample, UsbPassthroughMode, HVPM};

#[derive(Parser, Clone)]
#[command(version, about, long_about = None)]
struct Args {
    /// Sets the voltage and enables the power supply output.
    #[arg(long)]
    set_voltage: Option<f64>,

    /// Sets the USB passthrough mode.
    #[arg(long, value_enum, default_value_t = UsbPassthroughMode::Off)]
    usb_passthrough: UsbPassthroughMode,

    /// Keeps the output enabled when the program exits. Defaults to false
    #[arg(long, default_value_t = false)]
    keep_output_enabled: bool,

    /// Starts sampling for the given duration in seconds.
    #[arg(long, default_value_t = 0)]
    sampling_duration: u64,

    /// Aggregates collected samples per seconds. Defaults to false
    #[arg(long, default_value_t = false)]
    aggregate_samples: bool,
}

fn main() -> Result<(), rusb::Error> {
    env_logger::init();
    let args = Args::parse();
    let mut device = HVPM::new()?;
    device.set_usb_passthrough(args.usb_passthrough)?;
    if let Some(voltage) = args.set_voltage {
        device.set_vout(voltage)?;
    }
    if args.sampling_duration > 0 {
        device.start_sampling(Some(Duration::from_secs(args.sampling_duration as u64)))?;
        device.capture_samples()?;
        let (collected, dropped) = device.captured_samples();
        let samples: Vec<SoftwareSample> = if args.aggregate_samples {
            let collection_start = collected[0].timestamp;
            collected
                .into_iter()
                .group_by(|d| {
                    d.timestamp
                        .duration_since(collection_start)
                        .unwrap()
                        .as_secs()
                })
                .into_iter()
                .map(|(_k, g)| g.collect::<Vec<SoftwareSample>>())
                .map(|g| g.iter().sum::<SoftwareSample>() / g.len() as f64)
                .collect()
        } else {
            collected
        };
        println!("{}", SoftwareSample::csv_header());
        for s in samples {
            println!("{}", s.csv_row())
        }
    }
    if !args.keep_output_enabled {
        device.set_vout(0.)?;
    }
    Ok(())
}