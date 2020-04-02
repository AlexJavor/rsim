
extern crate nalgebra as na;
use structopt::StructOpt;


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
use crate::robot::{Position2D, Command};
use crate::robot::Veloc;
use ggez::{graphics, GameResult, Context, event, GameError};
use ggez::graphics::{DrawParam};
use std::ffi::OsStr;
use std::f64::consts::PI;
use crate::view::Meshes;
use crate::messages::{PointCloud, Point};
use crate::pubsub::Fluent;
use ggez::event::MouseButton;
use std::time::Duration;

struct Map {
    img: ggez::graphics::Image,
    pixel_size: f32,
}

pub enum SimResult {
    Success(f64),
    Crash,
    Timeout
}

struct MainState {
    env: Env<World2D>,
    meshes: Meshes,
    cloud: Fluent<PointCloud>,
    latest_command: Fluent<Command>,
    latest_target: Fluent<Point>,
    map: Map,
    conf: Opt,
    result: Option<SimResult>
}



impl MainState {
    fn new(ctx: &mut Context, conf: Opt) -> GameResult<MainState> {
        let map_img = ggez::graphics::Image::new(ctx, "/".to_owned() + &conf.map)
            .expect(format!("Could not find map file: {}", conf.map).as_str());
        let map = Map {
            img: map_img,
            pixel_size: 0.1f32
        };
        let world = World2D::new(ctx,Some(&map));
        let mut env = Env {
            pubsub: crate::pubsub::PubSub::init(),
            world,
            robots: Vec::new(),
            sensors: Vec::new(),
            actuators: Vec::new()
        };

        let rb_desc = RigidBodyDesc::new()
            .rotation(PI / 4. * 0.)
            .angular_inertia(10.)
            .mass(1.2);

        let bob_body = rb_desc.clone().translation(Vector2::new(10.0, 10.0)).build();
        let ball_shape = ShapeHandle::new(Ball::new(1.));

        let bob = env.add_robot("bob".into(), bob_body, ball_shape.clone(), conf.clone());

        let controller = unsafe { robot::plugin::Plugin::new(OsStr::new(&conf.controller)) };
        robot::wire(&mut env.pubsub, &bob, controller);

        let meshes = crate::view::Meshes::new(ctx).unwrap();
        let cloud = env.pubsub.last_value_cell(&bob.scan_topic).unwrap();
        let latest_command = env.pubsub.last_value_cell(&bob.command_topic).unwrap();
        let latest_target = env.pubsub.last_value_cell(&bob.target_topic).unwrap();

        {
            // this should come after wiring and after creating latest target so that the goal is accounted for
            // todo: support latched topics
            let target_setter = env.pubsub.poster(&bob.target_topic).unwrap();
            let target = Point { x: conf.target_x as f32, y: conf.target_y as f32};
            target_setter.send(target).unwrap();
        }


        let s = MainState {
          env: env,
            meshes,
            cloud,
            latest_command,
            latest_target,
            map,
            result: None,
            conf: conf.clone()
        };
        Ok(s)
    }
}

impl event::EventHandler for MainState {
    fn update(&mut self, ctx: &mut Context) -> Result<(), GameError> {
        //while !ggez::timer::check_update_time(ctx,60) {
            std::thread::sleep(Duration::from_micros(3000));
        //}

        self.env.step();

        let bob = &self.env.robots[0];

        let contacts = self.env.world.geometrical_world.contact_events();
        for contact in contacts {
            match contact {
                ncollide2d::narrow_phase::ContactEvent::Started(h1, h2) => if *h1 == bob.collider || *h2 == bob.collider {
                    self.result.replace(SimResult::Crash);
                    ggez::event::quit(ctx);
                },
                ncollide2d::narrow_phase::ContactEvent::Stopped(_, _) => ()
            }
        }

        let runtime = self.env.world.mechanical_world.integration_parameters.t;

        if let Some(target) = self.latest_target.get() {
            let pose = self.env.world.bodies.rigid_body(bob.body_handle).unwrap();
            let dist = f64::sqrt( (pose.x() - target.x as f64).powi(2) + (pose.y() - target.y as f64).powi(2));
            if dist < 0.2 {
                self.result.replace(SimResult::Success(runtime));
                ggez::event::quit(ctx);
            }
        }

        if runtime > self.conf.timeout {
            self.result.replace(SimResult::Timeout);
            ggez::event::quit(ctx);
        }

        Ok(())
    }

    fn draw(&mut self, ctx: &mut Context) -> Result<(), GameError> {
        let place = |pt: [f32;2]| { [pt[0]*10f32, pt[1] * 10f32] };

        graphics::clear(ctx, [0.1, 0.2, 0.3, 1.0].into());
        graphics::set_drawable_size(ctx, self.map.img.width() as f32, self.map.img.height() as f32)?;
        graphics::draw(ctx, &self.map.img, DrawParam::new())?;

        let bob = &self.env.robots[0];
        let body = self.env.world.bodies.rigid_body(bob.body_handle).unwrap();

        let mut text_lines = vec![
            format!("Pos: (x: {:.2}, y: {:.2}, theta: {:.2})", body.x(), body.y(), body.theta()),
            format!("Vel: (forward: {:.2}, lateral: {:.2}, rot: {:.2})", body.forward_vel(), body.lateral_vel(), body.rotational_vel())
        ];
        if let Some(cmd) = self.latest_command.get() {
            text_lines.push(
                format!("Command: (forward_vel: {:.2}, steering: {:.2})", cmd.forward_velocity, cmd.steering_angle)
            );
        }
        if let Some(target) = self.latest_target.get() {
            let d = f32::sqrt( (target.x - body.x() as f32).powi(2) + (target.y - body.y() as f32).powi(2));
            text_lines.push(
                format!("Target: ({:.2}, {:.2}) -- dist: {:.2}", target.x, target.y, d)
            );
            graphics::draw(ctx, &self.meshes.target,
                           DrawParam::new()
                               .dest(place([target.x, target.y]))
            ).unwrap();
        }
        for (i, l) in text_lines.iter().enumerate() {
            let display = graphics::Text::new(l.as_str());
            let pos = Point2::new(2., (i as f32) * 20.);
            graphics::draw(ctx, &display, (pos, 0.0, graphics::BLACK))?;
        }



        for r in &self.env.robots {
            let body = self.env.world.bodies.rigid_body(r.body_handle).unwrap();
            graphics::draw(ctx, &self.meshes.robot,
                           DrawParam::new()
                               .rotation(body.theta() as f32)
                               .dest(place([body.x() as f32, body.y() as f32]))
            ).unwrap();

        }

        if let Some(points) = self.cloud.get() {
            for p in points.points {
                graphics::draw(ctx, &self.meshes.impact, DrawParam::new().dest(place(p.into())))?;
            }
        }

        graphics::present(ctx)
    }

    fn mouse_button_up_event(
        &mut self,
        _ctx: &mut Context,
        _button: MouseButton,
        x: f32,
        y: f32,
    ) {
        let x = x / 10.; // TODO: properly handle transformations between view and simu
        let y = y / 10.;
        println!("Changing target to ({:.2}, {:.2})", x, y);
        let rob = &self.env.robots[0];
        let target_poster = self.env.pubsub.poster(&rob.target_topic)
            .expect("Could not initialize publisher to target topic");
        target_poster.send(Point { x: x, y: y })
            .expect("Could not publish target")
    }
}



#[derive(Debug, StructOpt, Clone)]
#[structopt(name = "rsim", about = "A simple 2D robotic simulator.")]
pub struct Opt {
    /// Path to an image containing the map
    #[structopt(short, long, default_value = "maps/base.jpg")]
    map: String,

    /// Pash to a shared library implenting the "controller.h" interface
    #[structopt(short, long, default_value = "controller/libdummy_controller.so")]
    controller: String,

    #[structopt(long, default_value = "2")]
    max_speed: f64,

    #[structopt(long, default_value = "0.5")]
    max_backward_speed: f64,

    #[structopt(long, default_value = "0.4")]
    max_rotation: f64,

    /// If set, the robot can turn in place
    #[structopt(long)]
    in_place: bool,

    /// X coordinate of the target to reach
    #[structopt(long, default_value = "50")]
    target_x: f64,

    /// Y coordinate of the target to reach
    #[structopt(long, default_value = "50")]
    target_y: f64,

    /// Timeout in seconds
    #[structopt(short, long, default_value = "60")]
    timeout: f64
}

fn main() -> ggez::error::GameResult {

    let conf = Opt::from_args();

    let context_builder = ggez::ContextBuilder::new("rsim", "Arthur Bit-Monnot")
        .add_resource_path(std::path::PathBuf::from("."))
        ;
    let (ctx, event_loop) = &mut context_builder.build()?;


    let state = &mut MainState::new(ctx, conf)?;
    event::run(ctx, event_loop, state)?;

    if let Some(res) = state.result.as_ref() {
        match res {
            SimResult::Success(t) => {
                println!("SUCCESS !!! Time to reach the target : {:.2}s", t);
            },
            SimResult::Crash => {
                println!("CRASH !!!!");
            },
            SimResult::Timeout => {
                println!("TIMEOUT !!!!")
            }
        }
    } else {
        println!("Simulation exited prematuraly");
    }

    Ok(())
}

