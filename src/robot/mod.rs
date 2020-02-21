pub mod plugin;

use nphysics2d::object::{RigidBody, DefaultBodyHandle, DefaultColliderHandle};
use crate::pubsub::{TopicDesc, PubSub, Fluent};
use ncollide2d::query::Ray;
use ncollide2d::pipeline::CollisionGroups;
use crate::messages::{Pose, Vel, Point, PointCloud};
use crate::env::{World2D, Sensor, Actuator};

use crate::utils::*;
use std::f64::consts::PI;
use std::f64::INFINITY;
use crate::robot::plugin::{Plugin, ControllerPlugin};
use std::sync::Arc;
use std::time::Duration;

#[allow(dead_code)]
pub struct Robot {
    acc_lim_x: f64,
    acc_lim_y: f64,
    acc_lim_theta: f64,
    max_vel_x: f64,
    min_vel_x: f64,
    max_vel_theta: f64,
    min_vel_theta: f64,
    min_in_place_vel_theta: f64,
    escape_vel: f64,
    holonomic_robot: bool,
    y_vels: [f64; 4],
}

impl Default for Robot {
    fn default() -> Self {
        Robot {
            acc_lim_x: 2.5,
            acc_lim_y: 2.5,
            acc_lim_theta: 3.2,
            max_vel_x: 0.5,
            min_vel_x: 0.1,
            max_vel_theta: 1.0,
            min_vel_theta: -1.0,
            min_in_place_vel_theta: 0.4,
            escape_vel: 0.1,
            holonomic_robot: false,
            y_vels: [-0.3, -0.1, 0.1, 0.3]
        }
    }
}

pub trait Position2D<R> {
    fn x(&self) -> R;
    fn y(&self) -> R;
    fn theta(&self) -> R;
}

impl<R: na::RealField> Position2D<R> for RigidBody<R> {
    fn x(&self) -> R {
        self.position().translation.x
    }

    fn y(&self) -> R {
        self.position().translation.y
    }

    fn theta(&self) -> R {
        self.position().rotation.angle()
    }
}

pub trait Veloc<R> {
    fn forward_vel(&self) -> R;
    fn lateral_vel(&self) -> R;
    fn rotational_vel(&self) -> R;
}

impl <R: na::RealField> Veloc<R> for RigidBody<R> {
    fn forward_vel(&self) -> R {
        let theta = self.theta();
        let dot_x = self.velocity().linear[0];
        let dot_y = self.velocity().linear[1];
        let v_x = dot_x * theta.cos();
        let v_y = dot_y * theta.sin();
        v_x + v_y
    }

    fn lateral_vel(&self) -> R {
        let dot_x = self.velocity().linear[0];
        let dot_y = self.velocity().linear[1];
        let v_x = dot_x * self.theta().sin();
        let v_y = - dot_y * self.theta().cos();
        v_x + v_y
    }

    fn rotational_vel(&self) -> R {
        self.velocity().angular
    }
}

#[derive(Copy, Clone, Debug, Default)]
#[repr(C)]
pub struct Command {
    pub forward_velocity: f32,
    pub steering_angle: f32
}


#[derive(Clone)]
pub struct BotDesc {
    pub id: String,
    pub body_handle: DefaultBodyHandle,
    pub collider: DefaultColliderHandle,
    pub pose_topic: TopicDesc<Pose>,
    pub vel_topic: TopicDesc<Vel>,
    pub command_topic: TopicDesc<Command>,
    pub scan_topic: TopicDesc<PointCloud>,
    pub target_topic: TopicDesc<Point>
}


impl BotDesc {
    pub fn new(id: String, body_handle: DefaultBodyHandle, collider: DefaultColliderHandle) -> Self {

        let pose_topic = TopicDesc::new(id.clone() + "/pose");
        let vel_topic = TopicDesc::new(id.clone()+ "/vel");
        let command_topic = TopicDesc::new(id.clone() + "/command");
        let scan_topic = TopicDesc::new(id.clone() + "/scan");
        let target_topic = TopicDesc::new(id.clone() + "/target");

        BotDesc {
            id,
            body_handle,
            collider,
            pose_topic,
            vel_topic,
            command_topic,
            scan_topic,
            target_topic
        }
    }
}

impl crate::env::Simulated<World2D> for BotDesc {

    fn sensors(&self, pubsub: &mut PubSub) -> Vec<Sensor<World2D>> {
        let pose_pub = pubsub.poster(&self.pose_topic).unwrap();
        let handle = self.body_handle.clone();
        let sense_pose :  Box<dyn Fn(&World2D)> =  Box::new(move |world: &World2D| {
            let body = world.bodies.rigid_body(handle).unwrap();
            pose_pub.send(Pose::from(*body.position())).log()
        });

        let self_collider = self.collider;
        let scan_pub = pubsub.poster(&self.scan_topic).unwrap();

        let lidar :  Box<dyn Fn(&World2D)> =  Box::new(move |world: &World2D| {
            let body = world.bodies.rigid_body(handle).unwrap();
            let position = body.position();
            let num_rays = 200;
            let angle_inc = PI * 2. / num_rays as f64;
            let mut angles = Vec::with_capacity(num_rays);
            for i in 0..num_rays {
                angles.push(i as f64 * angle_inc);
            }
            let mut points = Vec::with_capacity(num_rays);
            for &angle in &angles {
                let ray = Ray::new(
                    na::Point2::new(position.translation.vector[0], position.translation.vector[1]),
                    na::Vector2::new(angle.cos(), angle.sin())
                );
                let collision_group = CollisionGroups::new();
                let result = world.geometrical_world.interferences_with_ray(
                    &world.colliders,
                    &ray,
                    &collision_group
                );

                let mut min_toi = INFINITY;

                for (handle, _, inter) in result {
                    if handle != self_collider {
                        min_toi = min_toi.min(inter.toi);
                    }
                }
                if min_toi.is_finite() {
                    let pt = ray.point_at(min_toi);
                    points.push(Point { x: pt.x as f32, y: pt.y as f32});
                }
            }
            scan_pub.send(PointCloud { points }).log()
        });


        vec![
            Sensor { rate: 30.0, func: Box::new(sense_pose) },
            Sensor { rate: 5.0, func: Box::new(lidar)}
        ]
    }

    fn actuators(&self, pubsub: &mut PubSub) -> Vec<Actuator<World2D>> {
        let command_reader: Fluent<Command> = pubsub.last_value_cell(&self.command_topic).unwrap();
        let body_handle = self.body_handle.clone();
        let controller = move |world: &mut World2D| {
            let command = command_reader.get().unwrap_or_default();
            let body = world.bodies.rigid_body_mut(body_handle).unwrap();

            let v = command.forward_velocity;
            let theta = body.theta();
            let vx = v as f64 * theta.cos();
            let vy = v as f64 * theta.sin();
            let vtheta = v * command.steering_angle.tan() / 3.;
            body.set_linear_velocity(na::Vector2::new(vx, vy));
            body.set_angular_velocity(vtheta as f64);
        };
        vec![Box::new(controller)]
    }
}



pub fn wire(pubsub: &mut PubSub, bot: &BotDesc, controller: Plugin) {
    let plug = Arc::new(Box::new(controller));
    {
        let plug = plug.clone();
        pubsub.add_callback(&bot.pose_topic, Box::new( move |pose| plug.on_new_pose(pose) ))
            .log()
    }
    {
        let plug = plug.clone();
        pubsub.add_callback(&bot.scan_topic, Box::new( move |scan| plug.on_new_scan(scan) ))
            .log()
    }
    {
        let plug = plug.clone();
        pubsub.add_callback(&bot.target_topic, Box::new( move |target| plug.on_new_target(target)))
            .log()
    }
    {
        let plug = plug.clone();
        let poster = pubsub.poster(&bot.command_topic).unwrap();
        std::thread::spawn(move || {
            loop {
                std::thread::sleep(Duration::from_millis(30));
                let cmd = plug.get_command();
                poster.send(cmd).log();
            }
        });
    }
}