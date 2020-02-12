
extern crate nalgebra as na;
extern crate crossbeam;

mod robot;
mod pubsub;

use core::f32::consts::PI;

use na::Vector2;
use nphysics2d::object::{DefaultBodySet, DefaultColliderSet, BodyPartHandle, Body};
use nphysics2d::force_generator::DefaultForceGeneratorSet;
use nphysics2d::joint::DefaultJointConstraintSet;
use nphysics2d::world::{DefaultMechanicalWorld, DefaultGeometricalWorld};

use na::{Point2, Isometry2};
use nphysics2d::object::{BodyStatus, RigidBodyDesc};
use nphysics2d::math::{Velocity, Inertia};

use ncollide2d::shape::{ShapeHandle, Ball};
//use ncollide2d::world::CollisionGroups;
use nphysics2d::object::ColliderDesc;
use nphysics2d::material::{MaterialHandle, BasicMaterial};
use nphysics2d::algebra::ForceType::Force;
use nphysics2d::algebra::{Force2, ForceType};
use crate::robot::{Robot, Pos};


fn main() {
    let mut mechanical_world = DefaultMechanicalWorld::new(Vector2::new(0.0, 0.));
    let mut geometrical_world = DefaultGeometricalWorld::new();

    let mut bodies = DefaultBodySet::new();
    let mut colliders = DefaultColliderSet::new();
    let mut joint_constraints = DefaultJointConstraintSet::new();
    let mut force_generators = DefaultForceGeneratorSet::new();

    let mut rb_desc = RigidBodyDesc::new()
        .rotation(1.0)
        .mass(1.2);

    // Build a first rigid body.
    let rigid_body1 = rb_desc
        .set_angular_inertia(10.)
        .build();

    // Change the rigid body translation and velocity before building
    // another one. It will still have the same mass and rotation as
    // initialized above.
    let rigid_body2 = rb_desc.set_translation(Vector2::new(10.0, 0.0))
        .set_velocity(Velocity::linear(1.0, 3.0))
        .set_angular_inertia(10.)
        .build();

    let handle1 = bodies.insert(rigid_body1);
    let handle2 = bodies.insert(rigid_body2);

    let ball_shape = ShapeHandle::new(Ball::new(1.5));

    let handles = [handle1, handle2];

    for &h in handles.iter() {
        let collider = ColliderDesc::new(ball_shape.clone()).build(BodyPartHandle(h, 0));
        colliders.insert(collider);
    }
    let mut i = 0;
    loop {
        i += 1;
        if i > 10 {
            break;
        }

        let x = bodies.rigid_body(handle1).unwrap();
        println!("{:?}", x.velocity());
        println!(" {} {}", x.x(), x.y());
//        println!("{:?}", x.position());
        // Run the simulation.
        mechanical_world.step(
            &mut geometrical_world,
            &mut bodies,
            &mut colliders,
            &mut joint_constraints,
            &mut force_generators
        );

//        let f = Force2 {
//            linear: Vector2::new(60.,60.),
//            angular: 60.
//        };

        let target = robot::Command {
            forward_velocity: 1.6,
            steering_angle: 0.0
        };
        let body = bodies.rigid_body(handle1).unwrap();
        let f = robot::compute_acceleration(&target, body, &Robot::default()).expect("");
//        let tmp = f.as_vector();
//        tmp.cmpy();
        println!("{:?}", f);

        let x = bodies.rigid_body_mut(handle1).unwrap();
        x.apply_force(0, &f, ForceType::AccelerationChange, true);
//        x.set_angular_velocity(60.)
//        x.position().as_ref();

    }
}

