use nphysics2d::algebra::{Force2, ForceType};
use nphysics2d::object::{RigidBody, DefaultBodyHandle, DefaultColliderHandle};
use crate::pubsub::{TopicDesc, PubSub, Fluent};
use ncollide2d::query::Ray;
use ncollide2d::pipeline::CollisionGroups;
use crate::messages::{Pose, Vel};
use crate::env::{World2D, Sensor, Simulated, Actuator};

use crate::utils::*;

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

type R = f64;

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
pub struct Command {
    pub forward_velocity: R,
    pub steering_angle: R
}


#[derive(Clone)]
pub struct BotDesc {
    pub id: String,
    pub body_handle: DefaultBodyHandle,
    pub collider: DefaultColliderHandle,
    pub pose_topic: TopicDesc<Pose>,
    pub vel_topic: TopicDesc<Vel>,
    pub command_topic: TopicDesc<Command>
}


impl BotDesc {
    pub fn new(id: String, body_handle: DefaultBodyHandle, collider: DefaultColliderHandle) -> Self {

        let pose_topic = TopicDesc::new(id.clone() + "/pose");
        let vel_topic = TopicDesc::new(id.clone()+ "/vel");
        let command_topic = TopicDesc::new(id.clone() + "command");

        BotDesc {
            id,
            body_handle,
            collider,
            pose_topic,
            vel_topic,
            command_topic
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
        let id = self.id.clone();
        let lidar :  Box<dyn Fn(&World2D)> =  Box::new(move |world: &World2D| {
            let body = world.bodies.rigid_body(handle).unwrap();
            let position = body.position();
            let angle = position.rotation.angle();
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

            for (handle, _, inter) in result {
                if handle != self_collider {
                    println!("[{}]  ray impact: {}", id, ray.point_at(inter.toi));
                } else {
                    println!("[{}] self impact: {}", id, ray.point_at(inter.toi));
                }

            }
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
            let vx = v * theta.cos();
            let vy = v * theta.sin();
            let vtheta = v * command.steering_angle.tan() / 3.;
            body.set_linear_velocity(na::Vector2::new(vx, vy));
            body.set_angular_velocity(vtheta);
        };
        vec![Box::new(controller)]
    }
}
