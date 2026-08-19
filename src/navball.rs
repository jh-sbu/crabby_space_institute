use bevy::math::{DQuat, DVec2, DVec3};
use bevy_egui::egui;
use std::f64::consts::{PI, TAU};

use crate::model::Vessel;
use crate::orbit::body_definition;
use crate::simulation::maneuver_direction;

const BALL_DIAMETER: f32 = 218.0;
const BALL_RADIUS: f32 = BALL_DIAMETER * 0.5;
const MIN_GUIDANCE_SPEED_SQUARED: f64 = 0.25;

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub(crate) enum NavReference {
    #[default]
    Surface,
    Orbit,
}

#[derive(Debug, Clone, Copy)]
struct NavigationFrame {
    radial: DVec3,
    velocity: DVec3,
    prograde: Option<DVec3>,
    normal: Option<DVec3>,
}

pub(crate) fn show(
    ctx: &egui::Context,
    vessel: &Vessel,
    universal_time: f64,
    reference: &mut NavReference,
) {
    egui::Area::new(egui::Id::new("flight_navball"))
        .anchor(egui::Align2::CENTER_BOTTOM, egui::vec2(0.0, -14.0))
        .order(egui::Order::Middle)
        .show(ctx, |ui| {
            egui::Frame::new()
                .fill(egui::Color32::from_black_alpha(188))
                .stroke(egui::Stroke::new(
                    1.0,
                    egui::Color32::from_rgb(91, 109, 124),
                ))
                .corner_radius(12)
                .inner_margin(8)
                .show(ui, |ui| {
                    ui.set_width(BALL_DIAMETER + 18.0);
                    let frame = navigation_frame(vessel, *reference);
                    ui.vertical_centered(|ui| {
                        ui.label(
                            egui::RichText::new(format!(
                                "{:.1} m/s",
                                frame.velocity.length()
                            ))
                            .size(18.0)
                            .strong()
                            .color(egui::Color32::from_rgb(225, 238, 244)),
                        );
                        ui.horizontal(|ui| {
                            ui.selectable_value(reference, NavReference::Surface, "SURFACE");
                            ui.selectable_value(reference, NavReference::Orbit, "ORBIT");
                        });

                        let (rect, response) = ui.allocate_exact_size(
                            egui::vec2(BALL_DIAMETER, BALL_DIAMETER),
                            egui::Sense::hover(),
                        );
                        response.on_hover_text(
                            "Green: prograde / retrograde\nPurple: normal / anti-normal\nCyan: radial out / in\nGold: maneuver node",
                        );
                        paint_navball(
                            ui.painter(),
                            rect,
                            vessel,
                            universal_time,
                            frame,
                        );
                    });
                });
        });
}

fn navigation_frame(vessel: &Vessel, reference: NavReference) -> NavigationFrame {
    let radial = vessel.position_vec().normalize_or_zero();
    let velocity = reference_velocity(vessel, reference);
    let prograde =
        (velocity.length_squared() >= MIN_GUIDANCE_SPEED_SQUARED).then(|| velocity.normalize());
    let normal_vector = radial.cross(velocity);
    let normal = (normal_vector.length_squared() > f64::EPSILON).then(|| normal_vector.normalize());
    NavigationFrame {
        radial,
        velocity,
        prograde,
        normal,
    }
}

fn reference_velocity(vessel: &Vessel, reference: NavReference) -> DVec3 {
    match reference {
        NavReference::Surface => {
            let body = body_definition(&vessel.primary_body);
            vessel.velocity_vec() - ground_velocity(vessel.position_vec(), body.rotation_period)
        }
        NavReference::Orbit => vessel.velocity_vec(),
    }
}

fn ground_velocity(position: DVec3, rotation_period: f64) -> DVec3 {
    if rotation_period <= 0.0 {
        DVec3::ZERO
    } else {
        (DVec3::Z * (TAU / rotation_period)).cross(position)
    }
}

fn paint_navball(
    painter: &egui::Painter,
    rect: egui::Rect,
    vessel: &Vessel,
    universal_time: f64,
    frame: NavigationFrame,
) {
    let center = rect.center();
    let attitude = vessel.attitude_quat().normalize();

    painter.circle_filled(
        center,
        BALL_RADIUS + 5.0,
        egui::Color32::from_rgb(22, 29, 35),
    );
    painter.add(egui::Shape::mesh(ball_mesh(
        center,
        BALL_RADIUS,
        attitude,
        frame.radial,
    )));
    paint_grid(painter, center, BALL_RADIUS, attitude, frame.radial);

    if let Some(prograde) = frame.prograde {
        paint_paired_markers(
            painter,
            center,
            attitude,
            prograde,
            MarkerKind::Prograde,
            MarkerKind::Retrograde,
        );
    }
    if let Some(normal) = frame.normal {
        paint_paired_markers(
            painter,
            center,
            attitude,
            normal,
            MarkerKind::Normal,
            MarkerKind::AntiNormal,
        );
    }
    if frame.radial.length_squared() > f64::EPSILON {
        paint_paired_markers(
            painter,
            center,
            attitude,
            frame.radial,
            MarkerKind::RadialOut,
            MarkerKind::RadialIn,
        );
    }
    if let Some(direction) = maneuver_direction(vessel, universal_time) {
        let (offset, behind) = project_or_clamp(attitude, direction);
        let position = center + dvec2_to_vec2(offset) * BALL_RADIUS * 0.91;
        paint_marker(painter, position, MarkerKind::Maneuver, behind);
    }

    paint_reticle(painter, center);
    painter.circle_stroke(
        center,
        BALL_RADIUS,
        egui::Stroke::new(3.0, egui::Color32::from_rgb(132, 150, 160)),
    );
    painter.circle_stroke(
        center,
        BALL_RADIUS + 4.0,
        egui::Stroke::new(2.0, egui::Color32::from_rgb(51, 64, 73)),
    );
}

fn ball_mesh(center: egui::Pos2, radius: f32, attitude: DQuat, radial: DVec3) -> egui::Mesh {
    const RINGS: usize = 14;
    const SEGMENTS: usize = 96;

    let mut mesh = egui::Mesh::default();
    let forward = attitude * DVec3::Y;
    mesh.colored_vertex(center, ball_color(forward.dot(radial), 1.0));

    for ring in 1..=RINGS {
        let ring_radius = ring as f64 / RINGS as f64;
        let forward_component = (1.0 - ring_radius * ring_radius).max(0.0).sqrt();
        for segment in 0..SEGMENTS {
            let angle = TAU * segment as f64 / SEGMENTS as f64;
            let screen = DVec2::new(angle.cos(), angle.sin()) * ring_radius;
            let local_direction = DVec3::new(screen.x, forward_component, -screen.y);
            let world_direction = attitude * local_direction;
            mesh.colored_vertex(
                center + dvec2_to_vec2(screen) * radius,
                ball_color(world_direction.dot(radial), forward_component),
            );
        }
    }

    for segment in 0..SEGMENTS {
        mesh.add_triangle(
            0,
            mesh_index(1, segment, SEGMENTS),
            mesh_index(1, (segment + 1) % SEGMENTS, SEGMENTS),
        );
    }
    for ring in 2..=RINGS {
        for segment in 0..SEGMENTS {
            let next = (segment + 1) % SEGMENTS;
            let inner_a = mesh_index(ring - 1, segment, SEGMENTS);
            let inner_b = mesh_index(ring - 1, next, SEGMENTS);
            let outer_a = mesh_index(ring, segment, SEGMENTS);
            let outer_b = mesh_index(ring, next, SEGMENTS);
            mesh.add_triangle(inner_a, outer_a, inner_b);
            mesh.add_triangle(inner_b, outer_a, outer_b);
        }
    }
    mesh
}

fn mesh_index(ring: usize, segment: usize, segments: usize) -> u32 {
    (1 + (ring - 1) * segments + segment) as u32
}

fn ball_color(up_dot: f64, forward_component: f64) -> egui::Color32 {
    let base = if up_dot >= 0.0 {
        [38.0, 112.0, 169.0]
    } else {
        [157.0, 94.0, 48.0]
    };
    let limb_shade = 0.58 + 0.42 * forward_component.clamp(0.0, 1.0);
    egui::Color32::from_rgb(
        (base[0] * limb_shade) as u8,
        (base[1] * limb_shade) as u8,
        (base[2] * limb_shade) as u8,
    )
}

fn paint_grid(
    painter: &egui::Painter,
    center: egui::Pos2,
    radius: f32,
    attitude: DQuat,
    radial: DVec3,
) {
    if radial.length_squared() <= f64::EPSILON {
        return;
    }
    let (north, east) = tangent_basis(radial);
    for elevation_degrees in [-60.0_f64, -30.0, 0.0, 30.0, 60.0] {
        let elevation = elevation_degrees.to_radians();
        let stroke = if elevation_degrees == 0.0 {
            egui::Stroke::new(2.0, egui::Color32::from_rgb(235, 236, 221))
        } else {
            egui::Stroke::new(0.8, egui::Color32::from_white_alpha(115))
        };
        paint_world_curve(painter, center, radius, attitude, stroke, |step| {
            let azimuth = TAU * step;
            radial * elevation.sin()
                + (north * azimuth.cos() + east * azimuth.sin()) * elevation.cos()
        });
    }
    for heading in (0..360).step_by(30) {
        let azimuth = f64::from(heading).to_radians();
        let horizontal = north * azimuth.cos() + east * azimuth.sin();
        paint_world_curve(
            painter,
            center,
            radius,
            attitude,
            egui::Stroke::new(0.65, egui::Color32::from_white_alpha(75)),
            |step| {
                let elevation = -PI * 0.5 + PI * step;
                radial * elevation.sin() + horizontal * elevation.cos()
            },
        );
    }
}

fn tangent_basis(radial: DVec3) -> (DVec3, DVec3) {
    let up = radial.normalize_or_zero();
    let mut north = DVec3::Z - up * DVec3::Z.dot(up);
    if north.length_squared() <= 1.0e-12 {
        north = DVec3::X - up * DVec3::X.dot(up);
    }
    north = north.normalize_or_zero();
    let east = north.cross(up).normalize_or_zero();
    (north, east)
}

fn paint_world_curve(
    painter: &egui::Painter,
    center: egui::Pos2,
    radius: f32,
    attitude: DQuat,
    stroke: egui::Stroke,
    direction_at: impl Fn(f64) -> DVec3,
) {
    const STEPS: usize = 120;
    let mut previous: Option<egui::Pos2> = None;
    for step in 0..=STEPS {
        let direction = direction_at(step as f64 / STEPS as f64);
        let projected = project_direction(attitude, direction)
            .map(|offset| center + dvec2_to_vec2(offset) * radius);
        if let (Some(a), Some(b)) = (previous, projected)
            && a.distance(b) < radius * 0.2
        {
            painter.line_segment([a, b], stroke);
        }
        previous = projected;
    }
}

fn project_direction(attitude: DQuat, direction: DVec3) -> Option<DVec2> {
    if direction.length_squared() <= f64::EPSILON {
        return None;
    }
    let local = attitude.conjugate() * direction.normalize();
    (local.y >= 0.0).then_some(DVec2::new(local.x, -local.z))
}

fn project_or_clamp(attitude: DQuat, direction: DVec3) -> (DVec2, bool) {
    let local = attitude.conjugate() * direction.normalize_or_zero();
    let projected = DVec2::new(local.x, -local.z);
    if local.y >= 0.0 {
        (projected, false)
    } else if projected.length_squared() > f64::EPSILON {
        (projected.normalize(), true)
    } else {
        (DVec2::Y, true)
    }
}

fn paint_paired_markers(
    painter: &egui::Painter,
    center: egui::Pos2,
    attitude: DQuat,
    direction: DVec3,
    positive: MarkerKind,
    negative: MarkerKind,
) {
    if let Some(offset) = project_direction(attitude, direction) {
        paint_marker(
            painter,
            center + dvec2_to_vec2(offset) * BALL_RADIUS * 0.91,
            positive,
            false,
        );
    }
    if let Some(offset) = project_direction(attitude, -direction) {
        paint_marker(
            painter,
            center + dvec2_to_vec2(offset) * BALL_RADIUS * 0.91,
            negative,
            false,
        );
    }
}

#[derive(Debug, Clone, Copy)]
enum MarkerKind {
    Prograde,
    Retrograde,
    Normal,
    AntiNormal,
    RadialOut,
    RadialIn,
    Maneuver,
}

fn paint_marker(painter: &egui::Painter, position: egui::Pos2, kind: MarkerKind, behind: bool) {
    let green = egui::Color32::from_rgb(85, 238, 119);
    let purple = egui::Color32::from_rgb(225, 109, 244);
    let cyan = egui::Color32::from_rgb(78, 224, 239);
    let gold = egui::Color32::from_rgb(255, 190, 54);
    match kind {
        MarkerKind::Prograde => {
            let stroke = egui::Stroke::new(2.0, green);
            painter.circle_stroke(position, 7.0, stroke);
            painter.circle_filled(position, 2.0, green);
            for angle in [-PI * 0.5, PI / 6.0, PI * 5.0 / 6.0] {
                let direction = egui::vec2(angle.cos() as f32, angle.sin() as f32);
                painter.line_segment(
                    [position + direction * 7.0, position + direction * 11.0],
                    stroke,
                );
            }
        }
        MarkerKind::Retrograde => {
            let stroke = egui::Stroke::new(2.0, green);
            painter.circle_stroke(position, 7.0, stroke);
            painter.line_segment(
                [
                    position + egui::vec2(-5.0, -5.0),
                    position + egui::vec2(5.0, 5.0),
                ],
                stroke,
            );
            painter.line_segment(
                [
                    position + egui::vec2(-5.0, 5.0),
                    position + egui::vec2(5.0, -5.0),
                ],
                stroke,
            );
        }
        MarkerKind::Normal | MarkerKind::AntiNormal => {
            let sign = if matches!(kind, MarkerKind::Normal) {
                -1.0
            } else {
                1.0
            };
            let points = vec![
                position + egui::vec2(0.0, sign * 9.0),
                position + egui::vec2(-8.0, -sign * 6.0),
                position + egui::vec2(8.0, -sign * 6.0),
            ];
            painter.add(egui::Shape::convex_polygon(
                points,
                egui::Color32::from_rgba_unmultiplied(225, 109, 244, 45),
                egui::Stroke::new(2.0, purple),
            ));
            if matches!(kind, MarkerKind::AntiNormal) {
                painter.circle_filled(position, 2.0, purple);
            }
        }
        MarkerKind::RadialOut | MarkerKind::RadialIn => {
            let stroke = egui::Stroke::new(2.0, cyan);
            let points = [
                position + egui::vec2(0.0, -9.0),
                position + egui::vec2(9.0, 0.0),
                position + egui::vec2(0.0, 9.0),
                position + egui::vec2(-9.0, 0.0),
                position + egui::vec2(0.0, -9.0),
            ];
            for pair in points.windows(2) {
                painter.line_segment([pair[0], pair[1]], stroke);
            }
            if matches!(kind, MarkerKind::RadialOut) {
                painter.circle_filled(position, 2.2, cyan);
            } else {
                painter.line_segment(
                    [
                        position + egui::vec2(-4.0, -4.0),
                        position + egui::vec2(4.0, 4.0),
                    ],
                    stroke,
                );
                painter.line_segment(
                    [
                        position + egui::vec2(-4.0, 4.0),
                        position + egui::vec2(4.0, -4.0),
                    ],
                    stroke,
                );
            }
        }
        MarkerKind::Maneuver => {
            let stroke = egui::Stroke::new(2.2, gold);
            painter.circle_stroke(position, 8.0, stroke);
            painter.line_segment(
                [
                    position + egui::vec2(-12.0, 0.0),
                    position + egui::vec2(12.0, 0.0),
                ],
                stroke,
            );
            painter.line_segment(
                [
                    position + egui::vec2(0.0, -12.0),
                    position + egui::vec2(0.0, 12.0),
                ],
                stroke,
            );
            if behind {
                painter.circle_filled(position, 3.0, gold);
            }
        }
    }
}

fn paint_reticle(painter: &egui::Painter, center: egui::Pos2) {
    let stroke = egui::Stroke::new(2.5, egui::Color32::from_rgb(255, 216, 104));
    painter.line_segment(
        [
            center + egui::vec2(-28.0, 0.0),
            center + egui::vec2(-7.0, 0.0),
        ],
        stroke,
    );
    painter.line_segment(
        [
            center + egui::vec2(7.0, 0.0),
            center + egui::vec2(28.0, 0.0),
        ],
        stroke,
    );
    painter.line_segment(
        [
            center + egui::vec2(-7.0, 0.0),
            center + egui::vec2(0.0, 6.0),
        ],
        stroke,
    );
    painter.line_segment(
        [center + egui::vec2(7.0, 0.0), center + egui::vec2(0.0, 6.0)],
        stroke,
    );
    painter.circle_filled(center, 2.4, egui::Color32::from_rgb(255, 216, 104));
}

fn dvec2_to_vec2(value: DVec2) -> egui::Vec2 {
    egui::vec2(value.x as f32, value.y as f32)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::{ManeuverNode, PartCatalog, stock_craft};
    use approx::assert_abs_diff_eq;

    fn stock_vessel() -> Vessel {
        Vessel::from_blueprint(&stock_craft(), &PartCatalog::default())
    }

    #[test]
    fn launchpad_surface_speed_is_zero_but_orbit_speed_is_not() {
        let vessel = stock_vessel();
        assert!(reference_velocity(&vessel, NavReference::Surface).length() < 1.0e-9);
        assert!(reference_velocity(&vessel, NavReference::Orbit).length() > 1.0);
        let frame = navigation_frame(&vessel, NavReference::Surface);
        assert!(frame.prograde.is_none());
        assert!(frame.normal.is_none());
    }

    #[test]
    fn projection_uses_local_y_as_forward_and_local_z_as_screen_up() {
        let attitude = DQuat::IDENTITY;
        assert_eq!(project_direction(attitude, DVec3::Y), Some(DVec2::ZERO));
        assert_eq!(project_direction(attitude, DVec3::X), Some(DVec2::X));
        assert_eq!(project_direction(attitude, DVec3::Z), Some(-DVec2::Y));
        assert_eq!(project_direction(attitude, -DVec3::Y), None);

        let pitched = DQuat::from_rotation_x(0.4);
        let center = project_direction(pitched, pitched * DVec3::Y).unwrap();
        assert_abs_diff_eq!(center.x, 0.0, epsilon = 1.0e-12);
        assert_abs_diff_eq!(center.y, 0.0, epsilon = 1.0e-12);
    }

    #[test]
    fn navigation_axes_are_orthogonal() {
        let mut vessel = stock_vessel();
        vessel.position = [1_000_000.0, 0.0, 0.0];
        vessel.velocity = [0.0, 1_000.0, 0.0];
        let frame = navigation_frame(&vessel, NavReference::Orbit);
        let prograde = frame.prograde.unwrap();
        let normal = frame.normal.unwrap();
        assert_abs_diff_eq!(frame.radial.dot(prograde), 0.0, epsilon = 1.0e-12);
        assert_abs_diff_eq!(frame.radial.dot(normal), 0.0, epsilon = 1.0e-12);
        assert_abs_diff_eq!(prograde.dot(normal), 0.0, epsilon = 1.0e-12);
    }

    #[test]
    fn maneuver_marker_uses_the_planned_delta_v_direction() {
        let mut vessel = stock_vessel();
        vessel.position = [1_000_000.0, 0.0, 0.0];
        vessel.velocity = [0.0, 1_000.0, 0.0];
        vessel.maneuver = Some(ManeuverNode {
            ut: 10.0,
            prograde: 25.0,
            normal: 0.0,
            radial: 0.0,
        });
        let direction = maneuver_direction(&vessel, 10.0).unwrap();
        assert_abs_diff_eq!(direction.x, 0.0, epsilon = 1.0e-12);
        assert_abs_diff_eq!(direction.y, 1.0, epsilon = 1.0e-12);
        assert_abs_diff_eq!(direction.z, 0.0, epsilon = 1.0e-12);

        vessel.maneuver.as_mut().unwrap().prograde = 0.0;
        assert!(maneuver_direction(&vessel, 10.0).is_none());
    }

    #[test]
    fn tangent_basis_stays_finite_at_the_poles() {
        for radial in [DVec3::Z, -DVec3::Z] {
            let (north, east) = tangent_basis(radial);
            assert!(north.is_finite());
            assert!(east.is_finite());
            assert_abs_diff_eq!(north.length(), 1.0, epsilon = 1.0e-12);
            assert_abs_diff_eq!(east.length(), 1.0, epsilon = 1.0e-12);
            assert_abs_diff_eq!(north.dot(east), 0.0, epsilon = 1.0e-12);
        }
    }
}
