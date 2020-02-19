
extern crate nalgebra as na;


mod robot;
mod pubsub;
mod env;
mod messages;
mod utils;
use crate::env::*;
use crate::utils::*;

use core::f64::consts::PI;

use na::{Vector2};
use na::{Point2};
use nphysics2d::object::{RigidBodyDesc};

use ncollide2d::shape::{ShapeHandle, Ball};
use crate::robot::{Position2D};
use crate::robot::Veloc;
use ggez::graphics;
use ggez::graphics::{DrawParam, DrawMode, Color};


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

    let command = env.pubsub.poster(&bob.command_topic).unwrap();


    let context_builder = ggez::ContextBuilder::new("goto", "Arthur Bit-Monnot");
    let (ctx, _event_loop) = &mut context_builder.build()?;

    let mut i = 0;
    loop {
        i += 1;
        if i > 60 * 20 {
            break;
        }

        env.step();

        graphics::clear(ctx, [0.1, 0.2, 0.3, 1.0].into());

        let body = env.world.bodies.rigid_body(bob.body_handle).unwrap();
        let mouse = ggez::input::mouse::position(ctx);

        let dx = mouse.x as f64 - body.x();
        let dy = mouse.y as f64- body.y();
        let angle_to_mouse = dy.atan2(dx);
        let angle_diff = PI - f64::abs(f64::abs(body.theta() - angle_to_mouse) - PI);

        let steering_angle = if angle_diff.abs() < 0.01 {
            0.
        } else if angle_diff < 0. {
            -0.4
        } else {
            0.4
        };

        let target = robot::Command {
            forward_velocity: 1.6,
            steering_angle
        };
        command.send(target).log();
        println!("{} {} {}", body.theta(), angle_to_mouse, angle_diff);
        println!("{:?}", mouse);



        let text_lines = [
            format!("Pos: (x: {:.2}, y: {:.2}, theta: {:.2})", body.x(), body.y(), body.theta()),
            format!("Vel: (forward: {:.2}, lateral: {:.2}, rot: {:.2})", body.forward_vel(), body.lateral_vel(), body.rotational_vel())
        ];
        for (i, l) in text_lines.iter().enumerate() {
            let display = graphics::Text::new(l.as_str());
            let pos = Point2::new(2., (i as f32) * 20.);
            graphics::draw(ctx, &display, (pos, 0.0, graphics::WHITE))?;
        }

        let mesh = &mut graphics::MeshBuilder::new();
        mesh.circle(
            DrawMode::fill(),
            [0f32, 0f32],
            10.,
            0.1,
            Color::new(1.0, 1.0, 1.0, 1.0),
        );
        mesh.line(
        &[[-2.5f32, 5f32],  [7f32, 0f32], [-2.5f32, -5f32]],
        3.0,
        Color::new(1.0, 0.0, 0.0, 1.0),
        )?;
        let mesh = mesh.build(ctx)?;

        for r in &env.robots {
            let body = env.world.bodies.rigid_body(r.body_handle).unwrap();
            graphics::draw(ctx, &mesh,
                           DrawParam::new()
                               .rotation(body.theta() as f32)
                               .dest([(10. * body.x()) as f32, (10. * body.y()) as f32])
            ).unwrap();

        }

        if i %2 == 0 {
            graphics::present(ctx)?;
        }
    }

    Ok(())
}

