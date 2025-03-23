This plugin implements a velocity damping system based on the spatial position, mass and linear and angular velocities of rigid bodies, using raycasting to derive the projection area and a system of linear and angular external impulses.

At the moment the plugin is not stable and is in the proof of concept stage and will most likely be abandoned, as I plan to create a more performant implementation with more features, such as air flows and aerodynamic body stabilization.

To use the plugin, you just need to register it after the Avian plugins and add the necessary components to the bodies that will participate in the simulation.
```rust
fn main() {
    App::new()
        .add_plugins((
            PhysicsPlugins::default(),
            SmartDampingPlugin,
        ))
        .add_systems(Startup, setup)
        .run();
}
```
```rust
fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        FloatingBody::default(),
        RigidBody::Dynamic,
        Transform::from_xyz(0.0, 2.0, 0.0),
        Collider::cuboid(1.0, 1.0, 0.1),
        Mass(1.0),
        AngularVelocity(Vec3::new(0.0, 0.0, 2.0)),
        Mesh3d(meshes.add(Cuboid::new(1.0, 1.0, 0.1))),
        MeshMaterial3d(materials.add(Color::WHITE)),
    ));
}
```

Video demonstrating the behavior of bodies with smart damping:
*Not yet*
