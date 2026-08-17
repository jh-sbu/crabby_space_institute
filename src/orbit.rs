use bevy::math::DVec3;
use serde::{Deserialize, Serialize};
use std::f64::consts::TAU;

use crate::model::{HOME_ATMOSPHERE, HOME_MU, HOME_RADIUS};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AtmosphereDef {
    pub height: f64,
    pub sea_level_density: f64,
    pub scale_height: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CelestialBodyDef {
    pub id: &'static str,
    pub name: &'static str,
    pub parent: Option<&'static str>,
    pub radius: f64,
    pub mu: f64,
    pub rotation_period: f64,
    pub semi_major_axis: f64,
    pub phase: f64,
    pub atmosphere: Option<AtmosphereDef>,
}

pub fn celestial_system() -> Vec<CelestialBodyDef> {
    vec![
        CelestialBodyDef {
            id: "pelagos",
            name: "Pelagos",
            parent: None,
            radius: 69_600_000.0,
            mu: 1.327e18,
            rotation_period: 2_160_000.0,
            semi_major_axis: 0.0,
            phase: 0.0,
            atmosphere: None,
        },
        CelestialBodyDef {
            id: "carapace",
            name: "Carapace",
            parent: Some("pelagos"),
            radius: HOME_RADIUS,
            mu: HOME_MU,
            rotation_period: 21_600.0,
            semi_major_axis: 13.6e9,
            phase: 0.0,
            atmosphere: Some(AtmosphereDef {
                height: HOME_ATMOSPHERE,
                sea_level_density: 1.225,
                scale_height: 8_500.0,
            }),
        },
        CelestialBodyDef {
            id: "selene",
            name: "Selene",
            parent: Some("carapace"),
            radius: 200_000.0,
            mu: 6.514e10,
            rotation_period: 138_000.0,
            semi_major_axis: 12.0e6,
            phase: 0.8,
            atmosphere: None,
        },
        CelestialBodyDef {
            id: "ferrum",
            name: "Ferrum",
            parent: Some("pelagos"),
            radius: 320_000.0,
            mu: 5.0e11,
            rotation_period: 40_000.0,
            semi_major_axis: 22.0e9,
            phase: 2.2,
            atmosphere: Some(AtmosphereDef {
                height: 35_000.0,
                sea_level_density: 0.18,
                scale_height: 6_000.0,
            }),
        },
    ]
}

pub fn body_definition(id: &str) -> CelestialBodyDef {
    celestial_system()
        .into_iter()
        .find(|body| body.id == id)
        .unwrap_or_else(|| {
            celestial_system()
                .into_iter()
                .find(|body| body.id == "carapace")
                .unwrap()
        })
}

/// Position and velocity of a body's center in the Pelagos inertial frame.
pub fn body_root_state(id: &str, ut: f64) -> (DVec3, DVec3) {
    let body = body_definition(id);
    let Some(parent_id) = body.parent else {
        return (DVec3::ZERO, DVec3::ZERO);
    };
    let parent = body_definition(parent_id);
    let (parent_position, parent_velocity) = body_root_state(parent_id, ut);
    let (relative_position, relative_velocity) = circular_ephemeris(&body, parent.mu, ut);
    (
        parent_position + relative_position,
        parent_velocity + relative_velocity,
    )
}

pub fn vessel_root_state(
    primary: &str,
    position: DVec3,
    velocity: DVec3,
    ut: f64,
) -> (DVec3, DVec3) {
    let (primary_position, primary_velocity) = body_root_state(primary, ut);
    (primary_position + position, primary_velocity + velocity)
}

pub fn root_to_body_state(body: &str, position: DVec3, velocity: DVec3, ut: f64) -> (DVec3, DVec3) {
    let (body_position, body_velocity) = body_root_state(body, ut);
    (position - body_position, velocity - body_velocity)
}

pub fn circular_ephemeris(body: &CelestialBodyDef, parent_mu: f64, ut: f64) -> (DVec3, DVec3) {
    if body.parent.is_none() {
        return (DVec3::ZERO, DVec3::ZERO);
    }
    let n = (parent_mu / body.semi_major_axis.powi(3)).sqrt();
    let angle = body.phase + n * ut;
    let (sin, cos) = angle.sin_cos();
    let position = DVec3::new(body.semi_major_axis * cos, body.semi_major_axis * sin, 0.0);
    let velocity = DVec3::new(
        -body.semi_major_axis * n * sin,
        body.semi_major_axis * n * cos,
        0.0,
    );
    (position, velocity)
}

#[derive(Debug, Clone, Copy, Default)]
pub struct OrbitalElements {
    pub semi_major_axis: f64,
    pub eccentricity: f64,
    pub inclination: f64,
    pub apoapsis: f64,
    pub periapsis: f64,
    pub period: Option<f64>,
    pub specific_energy: f64,
}

pub fn elements(position: DVec3, velocity: DVec3, mu: f64, body_radius: f64) -> OrbitalElements {
    let r = position.length();
    let v2 = velocity.length_squared();
    if r <= f64::EPSILON {
        return OrbitalElements::default();
    }
    let h = position.cross(velocity);
    let eccentricity_vector = velocity.cross(h) / mu - position / r;
    let eccentricity = eccentricity_vector.length();
    let specific_energy = v2 * 0.5 - mu / r;
    let semi_major_axis = if specific_energy.abs() > 1e-12 {
        -mu / (2.0 * specific_energy)
    } else {
        f64::INFINITY
    };
    let inclination = if h.length_squared() > 0.0 {
        (h.z / h.length()).clamp(-1.0, 1.0).acos()
    } else {
        0.0
    };
    let periapsis = semi_major_axis * (1.0 - eccentricity) - body_radius;
    let apoapsis = if eccentricity < 1.0 {
        semi_major_axis * (1.0 + eccentricity) - body_radius
    } else {
        f64::INFINITY
    };
    let period = (eccentricity < 1.0 && semi_major_axis.is_finite())
        .then(|| TAU * (semi_major_axis.powi(3) / mu).sqrt());
    OrbitalElements {
        semi_major_axis,
        eccentricity,
        inclination,
        apoapsis,
        periapsis,
        period,
        specific_energy,
    }
}

fn stumpff_c(z: f64) -> f64 {
    if z > 1e-8 {
        (1.0 - z.sqrt().cos()) / z
    } else if z < -1e-8 {
        ((-z).sqrt().cosh() - 1.0) / -z
    } else {
        0.5 - z / 24.0 + z * z / 720.0
    }
}

fn stumpff_s(z: f64) -> f64 {
    if z > 1e-8 {
        (z.sqrt() - z.sqrt().sin()) / z.powf(1.5)
    } else if z < -1e-8 {
        (((-z).sqrt()).sinh() - (-z).sqrt()) / (-z).powf(1.5)
    } else {
        1.0 / 6.0 - z / 120.0 + z * z / 5_040.0
    }
}

fn initial_universal_anomaly(
    position: DVec3,
    velocity: DVec3,
    mu: f64,
    dt: f64,
    alpha: f64,
) -> f64 {
    let r0 = position.length();
    let sqrt_mu = mu.sqrt();

    if alpha > 1e-8 {
        return sqrt_mu * dt * alpha;
    }

    if alpha < 0.0 {
        // Curtis, Orbital Mechanics for Engineering Students, Algorithm 3.3.
        // The elliptic estimate above is not a valid hyperbolic estimate: using
        // |alpha| there can send Newton's method far into the overflowing cosh
        // branch of the Stumpff functions.
        let a = alpha.recip();
        let direction = dt.signum();
        let log_argument = (-2.0 * mu * alpha * dt)
            / (position.dot(velocity) + direction * (-mu * a).sqrt() * (1.0 - r0 * alpha));
        if log_argument.is_finite() && log_argument > 0.0 {
            let estimate = direction * (-a).sqrt() * log_argument.ln();
            if estimate.is_finite() && estimate.signum() == direction {
                return estimate;
            }
        }
    }

    // A well-scaled, correctly signed estimate for near-parabolic states and
    // the rare hyperbolic geometry for which the logarithmic estimate is poor.
    sqrt_mu * dt / r0
}

fn solve_universal_anomaly(
    position: DVec3,
    velocity: DVec3,
    mu: f64,
    dt: f64,
    alpha: f64,
) -> Option<f64> {
    const MAX_ITERATIONS: usize = 64;
    const RELATIVE_STEP_TOLERANCE: f64 = 1e-12;

    let r0 = position.length();
    let vr0 = position.dot(velocity) / r0;
    let sqrt_mu = mu.sqrt();
    let mut x = initial_universal_anomaly(position, velocity, mu, dt, alpha);

    for _ in 0..MAX_ITERATIONS {
        let z = alpha * x * x;
        let c = stumpff_c(z);
        let s = stumpff_s(z);
        let value = r0 * vr0 / sqrt_mu * x * x * c + (1.0 - alpha * r0) * x * x * x * s + r0 * x
            - sqrt_mu * dt;
        let derivative =
            r0 * vr0 / sqrt_mu * x * (1.0 - z * s) + (1.0 - alpha * r0) * x * x * c + r0;
        let dx = value / derivative;
        if !dx.is_finite() {
            return None;
        }

        x -= dx;
        if !x.is_finite() {
            return None;
        }
        if dx.abs() <= RELATIVE_STEP_TOLERANCE * (1.0 + x.abs()) {
            return Some(x);
        }
    }

    None
}

/// Propagate a two-body state with the universal-variable f/g solution.
pub fn propagate_universal(position: DVec3, velocity: DVec3, mu: f64, dt: f64) -> (DVec3, DVec3) {
    if dt.abs() < f64::EPSILON {
        return (position, velocity);
    }
    let r0 = position.length();
    let alpha = 2.0 / r0 - velocity.length_squared() / mu;
    let sqrt_mu = mu.sqrt();
    let x = solve_universal_anomaly(position, velocity, mu, dt, alpha)
        .expect("universal-variable Kepler solver failed to converge");

    let z = alpha * x * x;
    let c = stumpff_c(z);
    let s = stumpff_s(z);
    let f = 1.0 - x * x / r0 * c;
    let g = dt - x * x * x / sqrt_mu * s;
    let next_position = f * position + g * velocity;
    let r = next_position.length();
    let fdot = sqrt_mu / (r * r0) * x * (z * s - 1.0);
    let gdot = 1.0 - x * x / r * c;
    let next_velocity = fdot * position + gdot * velocity;
    (next_position, next_velocity)
}

pub fn sample_trajectory(
    position: DVec3,
    velocity: DVec3,
    mu: f64,
    body_radius: f64,
    count: usize,
) -> Vec<DVec3> {
    let orbit = elements(position, velocity, mu, body_radius);
    let duration = orbit.period.unwrap_or(7_200.0).min(200_000.0);
    (0..=count)
        .map(|i| {
            propagate_universal(
                position,
                velocity,
                mu,
                duration * i as f64 / count.max(1) as f64,
            )
            .0
        })
        .take_while(|p| p.length() >= body_radius)
        .collect()
}

pub fn sphere_of_influence(semi_major_axis: f64, body_mu: f64, parent_mu: f64) -> f64 {
    semi_major_axis * (body_mu / parent_mu).powf(0.4)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn circular_orbit_returns_to_start() {
        let radius = HOME_RADIUS + 100_000.0;
        let velocity = (HOME_MU / radius).sqrt();
        let period = TAU * (radius.powi(3) / HOME_MU).sqrt();
        let (p, v) = propagate_universal(
            DVec3::new(radius, 0.0, 0.0),
            DVec3::new(0.0, velocity, 0.0),
            HOME_MU,
            period,
        );
        assert_relative_eq!(p.x, radius, epsilon = 0.05);
        assert_relative_eq!(p.y, 0.0, epsilon = 0.05);
        assert_relative_eq!(v.y, velocity, epsilon = 1e-5);
    }

    #[test]
    fn orbital_elements_report_expected_circular_altitude() {
        let radius = HOME_RADIUS + 80_000.0;
        let el = elements(
            DVec3::X * radius,
            DVec3::Y * (HOME_MU / radius).sqrt(),
            HOME_MU,
            HOME_RADIUS,
        );
        assert_relative_eq!(el.apoapsis, 80_000.0, epsilon = 1e-6);
        assert_relative_eq!(el.periapsis, 80_000.0, epsilon = 1e-6);
        assert!(el.eccentricity < 1e-12);
    }

    #[test]
    fn home_soi_contains_selene() {
        let soi = sphere_of_influence(13.6e9, HOME_MU, 1.327e18);
        assert!(soi > 12.0e6);
    }

    #[test]
    fn long_hyperbolic_propagation_is_finite_and_conserves_energy() {
        let radius = HOME_RADIUS + 100_000.0;

        for eccentricity in [1.21, 1.88, 3.5, 7.0] {
            let position = DVec3::X * radius;
            let velocity = DVec3::Y * (HOME_MU * (1.0 + eccentricity) / radius).sqrt();
            let initial_energy = velocity.length_squared() * 0.5 - HOME_MU / radius;

            for dt in [2_000.0, 7_200.0, 30_000.0, 86_400.0] {
                let (next_position, next_velocity) =
                    propagate_universal(position, velocity, HOME_MU, dt);
                let final_energy =
                    next_velocity.length_squared() * 0.5 - HOME_MU / next_position.length();

                assert!(next_position.is_finite());
                assert!(next_velocity.is_finite());
                assert_relative_eq!(
                    final_energy,
                    initial_energy,
                    max_relative = 2e-11,
                    epsilon = 1e-6
                );
            }
        }
    }

    #[test]
    fn hyperbolic_propagation_is_reversible() {
        let radius = HOME_RADIUS + 100_000.0;
        let position = DVec3::X * radius;
        let velocity = DVec3::Y * (HOME_MU * 4.5 / radius).sqrt();
        let (later_position, later_velocity) =
            propagate_universal(position, velocity, HOME_MU, 86_400.0);
        let (restored_position, restored_velocity) =
            propagate_universal(later_position, later_velocity, HOME_MU, -86_400.0);

        assert_relative_eq!(restored_position.x, position.x, epsilon = 1e-5);
        assert_relative_eq!(restored_position.y, position.y, epsilon = 1e-5);
        assert_relative_eq!(restored_velocity.x, velocity.x, epsilon = 2e-8);
        assert_relative_eq!(restored_velocity.y, velocity.y, epsilon = 2e-8);
    }

    #[test]
    fn sampled_hyperbolic_trajectory_keeps_moving_outward() {
        let radius = HOME_RADIUS + 100_000.0;
        let position = DVec3::X * radius;
        let velocity = DVec3::Y * (HOME_MU * 4.5 / radius).sqrt();
        let points = sample_trajectory(position, velocity, HOME_MU, HOME_RADIUS, 160);

        assert_eq!(points.len(), 161);
        assert!(points.iter().all(|point| point.is_finite()));
        assert!(
            points
                .windows(2)
                .all(|pair| pair[1].length() >= pair[0].length())
        );
    }
}
