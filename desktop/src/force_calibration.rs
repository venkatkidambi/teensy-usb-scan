use serde::{Deserialize, Serialize};
use serde_json::{Map, Value};

#[derive(Deserialize, Serialize)]
pub struct LinearFit {
    slope: f64,
    intercept: f64,
}

#[derive(Deserialize, Serialize)]
pub struct Calibration {
    lps_to_force_n: LinearFit,
    supported_lps_range: [f64; 2],
    #[serde(flatten)]
    provenance: Map<String, Value>,
}

impl Calibration {
    pub fn load() -> Self {
        serde_json::from_str(include_str!("../calibration/lps-force.json"))
            .expect("embedded force calibration must be valid")
    }

    pub fn newtons(&self, raw: i64) -> f64 {
        self.lps_to_force_n.slope * raw as f64 + self.lps_to_force_n.intercept
    }

    pub fn extrapolated(&self, raw: i64) -> bool {
        (raw as f64) < self.supported_lps_range[0] || (raw as f64) > self.supported_lps_range[1]
    }
}
