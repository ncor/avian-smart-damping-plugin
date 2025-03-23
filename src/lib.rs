use std::f32;

use avian3d::{dynamics::integrator::IntegrationSet, math::PI, prelude::*};
use bevy::{ecs::query::QueryData, prelude::*};

const TWO_PI: f32 = 2.0 * PI;
const FOUR_TRIPLES_PI: f32 = 4.0 / 3.0 * PI;
const DEFAULT_RAYS_PER_SIDE: u8 = 7;
const RAY_PLANE_DOWNSCALE: f32 = 0.95;
const DEFAULT_MINIMUM_SIGNIFICANT_LINEAR_VELOCITY: f32 = 0.015;
const DEFAULT_MINIMUM_SIGNIFICANT_ANGULAR_VELOCITY: f32 = 0.015;
const DEFAULT_MEDIUM_DENSITY: f32 = 1.225;
const DEFAULT_DRAG_COEFFICIENT: f32 = 0.5;

#[derive(Component)]
pub struct SmartDamping {
    pub contact_area_quality: u8,
    pub drag_coefficient: f32,
    pub minimum_significant_linear_velocity: f32,
    pub minimum_significant_angular_velocity: f32,
    pub is_sphere: bool,
}

impl Default for SmartDamping {
    fn default() -> Self {
        Self {
            contact_area_quality: DEFAULT_RAYS_PER_SIDE,
            drag_coefficient: DEFAULT_DRAG_COEFFICIENT,
            minimum_significant_linear_velocity: DEFAULT_MINIMUM_SIGNIFICANT_LINEAR_VELOCITY,
            minimum_significant_angular_velocity: DEFAULT_MINIMUM_SIGNIFICANT_ANGULAR_VELOCITY,
            is_sphere: false,
        }
    }
}

pub struct SmartDampingPlugin;

impl Plugin for SmartDampingPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            FixedPostUpdate,
            (integrate_linear_impulses, integrate_angular_impulses)
                .in_set(IntegrationSet::Velocity),
        );
    }
}

struct ProjectingBody<'c> {
    position: Vec3,
    rotation: Quat,
    collider: &'c Collider,
}

struct ProjectionPlane {
    origin: Vec3,
    size: Vec2,
    uv: Vec3,
}

fn area_of_raster_body_projection(
    body: &ProjectingBody,
    plane: &ProjectionPlane,
    minimum_rays_per_side: u8,
) -> f32 {
    let plane_y_basis = plane.uv.any_orthogonal_vector().normalize();
    let plane_x_basis = plane.uv.cross(plane_y_basis).normalize();

    let ray_plane_size = plane.size * RAY_PLANE_DOWNSCALE;
    let ray_plane_x_step = plane_x_basis * ray_plane_size.x;
    let ray_plane_y_step = plane_y_basis * ray_plane_size.y;

    let min_area_chunk_length = ray_plane_size.min_element() / minimum_rays_per_side as f32;
    let rays_on = (ray_plane_size / min_area_chunk_length).floor();
    let total_rays = rays_on.element_product();

    let mut ray_hit_count: i16 = 0;

    let mut ray_x_index: f32 = 0.0;
    while ray_x_index < rays_on.x {
        let position_x = ray_x_index / (rays_on.x - 1.0);

        let mut ray_y_index: f32 = 0.0;
        while ray_y_index < rays_on.y {
            let position_y = ray_y_index / (rays_on.y - 1.0);
            let origin = plane.origin
                + (ray_plane_x_step * (position_x - 0.5))
                + (ray_plane_y_step * (position_y - 0.5));

            if let Some(_) = body.collider.cast_ray(
                body.position,
                body.rotation,
                origin,
                plane.uv,
                f32::INFINITY,
                true,
            ) {
                ray_hit_count += 1;
            }

            ray_y_index += 1.0;
        }

        ray_x_index += 1.0;
    }

    ray_hit_count as f32 / total_rays * plane.size.element_product()
}

fn area_of_symmetric_raster_body_projections(
    body: &ProjectingBody,
    image_plane: &ProjectionPlane,
    minimum_rays_per_side: u8,
) -> f32 {
    area_of_raster_body_projection(body, image_plane, minimum_rays_per_side)
        + area_of_raster_body_projection(
            &body,
            &ProjectionPlane {
                origin: 2.0 * body.position - image_plane.origin,
                size: image_plane.size,
                uv: -image_plane.uv,
            },
            minimum_rays_per_side,
        )
}

#[inline]
fn sphere_volume(radius: f32) -> f32 {
    FOUR_TRIPLES_PI * radius.powf(3.0)
}

#[inline]
fn horn_torus_volume(cross_section_area: f32, cross_section_radius: f32) -> f32 {
    TWO_PI * cross_section_radius * cross_section_area
}

#[derive(QueryData)]
#[query_data(mutable)]
struct AngularImpulsesIntegrationQuery {
    smart_damping: &'static SmartDamping,
    position: &'static Position,
    rotation: &'static Rotation,
    collider: &'static Collider,
    mass: &'static ComputedMass,
    angular_velocity: &'static AngularVelocity,
    angular_inertia: &'static ComputedAngularInertia,
    angular_impulse: &'static mut ExternalAngularImpulse,
}

fn integrate_angular_impulses(time: Res<Time>, mut bodies: Query<AngularImpulsesIntegrationQuery>) {
    let delta_seconds = time.delta_secs();

    for mut body in bodies.iter_mut() {
        if body.angular_velocity.length() < body.smart_damping.minimum_significant_angular_velocity
        {
            continue;
        }

        let position = body.position.0;
        let rotation = body.rotation.0;
        let collider = body.collider;
        let angular_velocity = body.angular_velocity.0;
        let revolutions_per_second = angular_velocity / TWO_PI;

        let rotational_volumes = if body.smart_damping.is_sphere {
            Vec3::splat(sphere_volume(
                collider.aabb(position, rotation).size().length() / 2.0,
            ))
        } else {
            let x_rotation_torus_volume = {
                let rotation = Quat::from_xyzw(0.0, rotation.y, rotation.z, rotation.w);
                let aabb_size = collider.aabb(position, rotation).size();
                let projection_planes_height = aabb_size.y / 2.0;

                let contact_area = area_of_symmetric_raster_body_projections(
                    &ProjectingBody {
                        position,
                        rotation,
                        collider,
                    },
                    &ProjectionPlane {
                        origin: position
                            + Vec3::new(
                                0.0,
                                aabb_size.y / 4.0 * angular_velocity.x.signum(),
                                aabb_size.z / 2.0,
                            ),
                        size: Vec2::new(aabb_size.x, projection_planes_height),
                        uv: -Vec3::Z,
                    },
                    body.smart_damping.contact_area_quality,
                );

                horn_torus_volume(contact_area, projection_planes_height / 2.0)
            };

            let y_rotation_torus_volume = {
                let rotation = Quat::from_xyzw(rotation.x, 0.0, rotation.z, rotation.w);
                let aabb_size = collider.aabb(position, rotation).size();
                let projection_planes_height = aabb_size.y;

                let contact_area = area_of_symmetric_raster_body_projections(
                    &ProjectingBody {
                        position,
                        rotation,
                        collider,
                    },
                    &ProjectionPlane {
                        origin: position
                            + Vec3::new(
                                aabb_size.x / 2.0,
                                0.0,
                                aabb_size.z / 4.0 * angular_velocity.y.signum(),
                            ),
                        size: Vec2::new(projection_planes_height, aabb_size.z / 2.0),
                        uv: -Vec3::X,
                    },
                    body.smart_damping.contact_area_quality,
                );

                horn_torus_volume(contact_area, projection_planes_height / 2.0)
            };

            let z_rotation_torus_volume = {
                let rotation = Quat::from_xyzw(rotation.x, rotation.y, 0.0, rotation.w);
                let aabb_size = collider.aabb(position, rotation).size();
                let projection_planes_height = aabb_size.y / 2.0;

                let contact_area = area_of_symmetric_raster_body_projections(
                    &ProjectingBody {
                        position,
                        rotation,
                        collider,
                    },
                    &ProjectionPlane {
                        origin: position
                            + Vec3::new(
                                aabb_size.x / 2.0,
                                aabb_size.y / 4.0 * -angular_velocity.z.signum(),
                                0.0,
                            ),
                        size: Vec2::new(projection_planes_height, aabb_size.z),
                        uv: -Vec3::X,
                    },
                    body.smart_damping.contact_area_quality,
                );

                horn_torus_volume(contact_area, projection_planes_height / 2.0)
            };

            Vec3::new(
                x_rotation_torus_volume,
                y_rotation_torus_volume,
                z_rotation_torus_volume,
            )
        };

        let volume_masses_moved_by_rotation =
            rotational_volumes * DEFAULT_MEDIUM_DENSITY * revolutions_per_second * delta_seconds;

        let response_angular_impulse =
            -(volume_masses_moved_by_rotation * body.smart_damping.drag_coefficient);

        let next_angular_velocity =
            angular_velocity + response_angular_impulse * body.mass.inverse();

        if next_angular_velocity.signum() == angular_velocity.signum() {
            body.angular_impulse.apply_impulse(response_angular_impulse);
        } else {
            // cancellation angular impulse
            body.angular_impulse
                .apply_impulse(body.angular_inertia.value() * -angular_velocity);
        }
    }
}

#[derive(QueryData)]
#[query_data(mutable)]
struct LinearImpulsesIntegrationQuery {
    smart_damping: &'static SmartDamping,
    position: &'static Position,
    rotation: &'static Rotation,
    collider: &'static Collider,
    mass: &'static ComputedMass,
    velocity: &'static LinearVelocity,
    impulse: &'static mut ExternalImpulse,
}

fn integrate_linear_impulses(time: Res<Time>, mut bodies: Query<LinearImpulsesIntegrationQuery>) {
    let delta_seconds = time.delta_secs();

    for mut body in bodies.iter_mut() {
        if body.velocity.length() < body.smart_damping.minimum_significant_linear_velocity {
            continue;
        }

        let position = body.position.0;
        let rotation = body.rotation.0;

        let velocity = body.velocity.0;
        let velocity_uv = velocity.normalize();
        let opposite_velocity_uv = -velocity_uv;

        let collider = body.collider;
        let aabb_size = collider.aabb(position, rotation).size();

        let contact_area = if body.smart_damping.is_sphere {
            PI * (aabb_size.length() / 2.0).powf(2.0)
        } else {
            let velocity_xy_uv = Vec2::new(velocity.x, velocity.y).normalize();

            let contact_plane_size = Vec2::new(
                aabb_size.x * velocity_xy_uv.y.abs() + aabb_size.y * velocity_xy_uv.x.abs(),
                aabb_size.z * (1.0 - velocity_uv.z.powf(2.0)).sqrt()
                    + velocity_uv.z.abs()
                        * (aabb_size.x * velocity_xy_uv.x.abs()
                            + aabb_size.y * velocity_xy_uv.y.abs()),
            );

            let contact_plane_origin =
                position + velocity_uv * ((velocity_uv.abs() * aabb_size).element_sum() / 2.0);

            area_of_raster_body_projection(
                &ProjectingBody {
                    position,
                    rotation,
                    collider,
                },
                &ProjectionPlane {
                    size: contact_plane_size,
                    origin: contact_plane_origin,
                    uv: opposite_velocity_uv,
                },
                body.smart_damping.contact_area_quality,
            )
        };

        let volume_mass_moved =
            contact_area * DEFAULT_MEDIUM_DENSITY * velocity.length() * delta_seconds;

        let response_impulse =
            volume_mass_moved * opposite_velocity_uv * body.smart_damping.drag_coefficient;

        let next_velocity = velocity + response_impulse * body.mass.inverse();

        if next_velocity.signum() == velocity_uv.signum() {
            body.impulse.apply_impulse(response_impulse);
        } else {
            // cancellation impulse
            body.impulse.apply_impulse(-velocity * body.mass.value());
        }
    }
}
