
extern crate nalgebra as na;


mod robot;
mod pubsub;
mod env;
mod messages;
mod utils;
mod view;

use crate::env::*;




use na::{Vector2};
use na::{Point2};
use nphysics2d::object::{RigidBodyDesc};

use ncollide2d::shape::{ShapeHandle, Ball};
use crate::robot::{Position2D};
use crate::robot::Veloc;
use ggez::graphics;
use ggez::graphics::{DrawParam};
use std::ffi::OsStr;



fn main() -> ggez::error::GameResult {

    let world = World2D::new();
    let mut env = Env {
        pubsub: crate::pubsub::PubSub::init(),
        world,
        robots: Vec::new(),
        sensors: Vec::new(),
        actuators: Vec::new()
    };

    let rb_desc = RigidBodyDesc::new()
        .rotation(3.14158 / 4. * 0.)
        .angular_inertia(10.)
        .mass(1.2);

    let bob_body = rb_desc.clone().translation(Vector2::new(10.0, 10.0)).build();
    let max_body = rb_desc.translation(Vector2::new(4.0, 20.0)).build();
    let ball_shape = ShapeHandle::new(Ball::new(1.));

    let bob = env.add_robot("bob".into(), bob_body, ball_shape.clone());
    let _max = env.add_robot("max".into(), max_body, ball_shape.clone());


    let controller = unsafe { robot::plugin::Plugin::new(OsStr::new("controller/libdummy_controller.so")) };
    robot::wire(&mut env.pubsub, &bob, controller);

    let cloud = env.pubsub.last_value_cell(&bob.scan_topic).unwrap();

    let context_builder = ggez::ContextBuilder::new("goto", "Arthur Bit-Monnot");
    let (ctx, _event_loop) = &mut context_builder.build()?;

    let meshes = crate::view::Meshes::new(ctx).unwrap();
    let latest_command = env.pubsub.last_value_cell(&bob.command_topic).unwrap();

    let mut i = 0;
    loop {
        i += 1;
        if i > 60 * 20 {
            break;
        }

        env.step();

        graphics::clear(ctx, [0.1, 0.2, 0.3, 1.0].into());

        let body = env.world.bodies.rigid_body(bob.body_handle).unwrap();

        let mut text_lines = vec![
            format!("Pos: (x: {:.2}, y: {:.2}, theta: {:.2})", body.x(), body.y(), body.theta()),
            format!("Vel: (forward: {:.2}, lateral: {:.2}, rot: {:.2})", body.forward_vel(), body.lateral_vel(), body.rotational_vel())
        ];
        if let Some(cmd) = latest_command.get() {
            text_lines.push(
                format!("Command: (forward_vel: {:.2}, steering: {:.2})", cmd.forward_velocity, cmd.steering_angle)
            );
        }
        for (i, l) in text_lines.iter().enumerate() {
            let display = graphics::Text::new(l.as_str());
            let pos = Point2::new(2., (i as f32) * 20.);
            graphics::draw(ctx, &display, (pos, 0.0, graphics::WHITE))?;
        }



        let place = |pt: [f32;2]| { [pt[0]*10f32, pt[1] * 10f32] };

        for r in &env.robots {
            let body = env.world.bodies.rigid_body(r.body_handle).unwrap();
            graphics::draw(ctx, &meshes.robot,
                           DrawParam::new()
                               .rotation(body.theta() as f32)
                               .dest(place([body.x() as f32, body.y() as f32]))
            ).unwrap();

        }

        if let Some(points) = cloud.get() {
            for p in points.points {

                graphics::draw(ctx, &meshes.impact, DrawParam::new().dest(place(p.into())))?;
            }
        }


        if i %2 == 0 {
            graphics::present(ctx)?;
        }
    }

    Ok(())
}

