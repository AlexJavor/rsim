
extern crate nalgebra as na;


mod robot;
mod pubsub;

use core::f32::consts::PI;

use na::{Vector2, Isometry, Unit, Complex};
use nphysics2d::object::{DefaultBodySet, DefaultColliderSet, BodyPartHandle, Body, RigidBody, DefaultBodyHandle, DefaultColliderHandle};
use nphysics2d::force_generator::DefaultForceGeneratorSet;
use nphysics2d::joint::DefaultJointConstraintSet;
use nphysics2d::world::{DefaultMechanicalWorld, DefaultGeometricalWorld, MechanicalWorld, GeometricalWorld};

use na::{Point2, Isometry2};
use nphysics2d::object::{BodyStatus, RigidBodyDesc};
use nphysics2d::math::{Velocity, Inertia};

use ncollide2d::shape::{ShapeHandle, Ball};
//use ncollide2d::world::CollisionGroups;
use nphysics2d::object::ColliderDesc;
use nphysics2d::material::{MaterialHandle, BasicMaterial};
use nphysics2d::algebra::ForceType::Force;
use nphysics2d::algebra::{Force2, ForceType, Velocity2};
use crate::robot::{Robot, Position2D, Command};
use crate::pubsub::{Fluent, TopicDesc, PubSub};
use ncollide2d::query::Ray;
use ncollide2d::pipeline::CollisionGroups;

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

impl Simulated<World2D> for BotDesc {

    fn sensors(&self, pubsub: &mut PubSub) -> Vec<Sensor<World2D>> {
        let pose_pub = pubsub.poster(&self.pose_topic).unwrap();
        let handle = self.body_handle.clone();
        let sense_pose :  Box<dyn Fn(&World2D)> =  Box::new(move |world: &World2D| {
            let body = world.bodies.rigid_body(handle).unwrap();
            pose_pub.send(Pose::from(*body.position()));
        });

        let self_collider = self.collider;
        let id = self.id.clone();
        let lidar :  Box<dyn Fn(&World2D)> =  Box::new(move |world: &World2D| {
            let body = world.bodies.rigid_body(handle).unwrap();
            let position = body.position();
            let angle = position.rotation.angle();
            let ray = Ray::new(
                Point2::new(position.translation.vector[0], position.translation.vector[1]),
                Vector2::new(angle.cos(), angle.sin())
            );
            let collision_group = CollisionGroups::new();
            let result = world.geometrical_world.interferences_with_ray(
                &world.colliders,
                &ray,
                &collision_group
            );

            for (handle, collider, inter) in result {
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

    fn controller(&self, pubsub: &mut PubSub) -> Vec<Controller<World2D>> {
        let command_reader: Fluent<Command> = pubsub.last_value_cell(&self.command_topic).unwrap();
        let body_handle = self.body_handle.clone();
        let controller = move |world: &mut World2D| {
            let command = command_reader.get().unwrap_or_default();
            let body = world.bodies.rigid_body_mut(body_handle).unwrap();
            let f = robot::compute_acceleration(&command, body, &Robot::default()).expect("");
            body.apply_force(0, &f, ForceType::AccelerationChange, true);
        };
        vec![Box::new(controller)]
    }
}

struct Sensor<World> {
    rate: f64,
    func: Box<dyn Fn(&World)>
}
type Controller<World>  = Box<dyn Fn(&mut World)>;

trait Simulated<World> {
    fn sensors(&self, pubsub: &mut PubSub) -> Vec<Sensor<World>>;
    fn controller(&self, pubsub: &mut PubSub) -> Vec<Controller<World>>;
}

trait Simu {
    fn step(&mut self);
}

#[derive(Copy, Clone, Default, Debug)]
pub struct Pose {
    pub x: f64,
    pub y: f64,
    pub theta: f64
}

impl From<Isometry2<f64>> for Pose {
    fn from(i: Isometry2<f64>) -> Self {
        Pose {
            x: i.translation.vector[0],
            y: i.translation.vector[1],
            theta: i.rotation.angle()
        }
    }
}

pub struct Vel {
    pub x: f64,
    pub y: f64,
    pub theta: f64
}


impl From<Velocity2<f64>> for Vel {
    fn from(v: Velocity2<f64>) -> Self {
        Vel {
            x: v.linear[0],
            y: v.linear[1],
            theta: v.angular
        }
    }
}

type N = f64;
type BodySet = DefaultBodySet<N>;
type BodyHandle = DefaultBodyHandle;
type ColliderHandle = DefaultColliderHandle;

struct World2D {
    mechanical_world: MechanicalWorld<N, BodySet, ColliderHandle>,
    geometrical_world: GeometricalWorld<N, BodyHandle, ColliderHandle>,
    bodies: BodySet,
    colliders: DefaultColliderSet<N>,
    joint_constraints: DefaultJointConstraintSet<N, BodySet>,
    force_generators: DefaultForceGeneratorSet<N, BodySet>
}

impl World2D {
    pub fn new() -> Self {
        World2D {
            mechanical_world: DefaultMechanicalWorld::new(Vector2::new(0.0, 0.)),
            geometrical_world: DefaultGeometricalWorld::new(),
            bodies: DefaultBodySet::new(),
            colliders: DefaultColliderSet::new(),
            joint_constraints: DefaultJointConstraintSet::new(),
            force_generators: DefaultForceGeneratorSet::new()
        }
    }
}


impl Simu for World2D {
    fn step(&mut self) {
        self.mechanical_world.step(
            &mut self.geometrical_world,
            &mut self.bodies,
            &mut self.colliders,
            &mut self.joint_constraints,
            &mut self.force_generators
        );
    }
}

struct Env<World> {
    pubsub: PubSub,
    world: World,
    robots: Vec<BotDesc>,
    sensors: Vec<(Sensor<World>, f64)>,
    actuators: Vec<Controller<World>>
}

impl<W> Env<W> {
    pub fn rate(&self) -> f64 {
        60.0
    }

}

impl Env<World2D> {


    pub fn add_robot(&mut self, id: String, body: RigidBody<N>, shape_handle: ShapeHandle<N>) -> BotDesc {
        let body_handle = self.world.bodies.insert(body);

        let mut collider = ColliderDesc::new(shape_handle).build(BodyPartHandle(body_handle, 0));
        collider.set_user_data(Some(Box::new(id.clone())));
        let collider_handle = self.world.colliders.insert(collider);

        let bot = BotDesc::new(id, body_handle, collider_handle);

        for sensor in bot.sensors(&mut self.pubsub) {
            self.sensors.push((sensor, 0.0));
        }
        for a in bot.controller(&mut self.pubsub) {
            self.actuators.push(a);
        }
        bot
    }

}

impl<World: Simu> Simu for Env<World> {
    fn step(&mut self) {
        self.world.step();

        let dt = 1.0 / 60.0; // TODO: fix that

        for (sensor, elapsed) in &mut self.sensors {
            *elapsed = *elapsed + dt;
            if *elapsed > (1.0 / sensor.rate - 1e-9) {
                *elapsed = 0.0;
                (sensor.func)(&self.world);
            }
        }

        for actu in &mut self.actuators {
            actu(&mut self.world);
        }
    }
}

fn main() {

    let mut world = World2D::new();
    let mut env = Env {
        pubsub: crate::pubsub::PubSub::init(),
        world,
        robots: Vec::new(),
        sensors: Vec::new(),
        actuators: Vec::new()
    };

    let mut rb_desc = RigidBodyDesc::new()
        .rotation(0.0)
        .angular_inertia(10.)
        .mass(1.2);

    let bob_body = rb_desc.build();
    let max_body = rb_desc.translation(Vector2::new(4.0, 0.0)).build();
    let ball_shape = ShapeHandle::new(Ball::new(1.));

    let bob = env.add_robot("bob".into(), bob_body, ball_shape.clone());
    let max = env.add_robot("max".into(), max_body, ball_shape.clone());

    for bot in &[&bob, &max] {
        let id = bot.id.clone();
        env.pubsub.add_callback(&bot.pose_topic, Box::new(move |pose| println!("{} : {:?}", id, pose)));
    }


    let command = env.pubsub.poster(&bob.command_topic).unwrap();

    let mut i = 0;
    loop {
        i += 1;
        if i > 20 {
            break;
        }

        env.step();

        let target = robot::Command {
            forward_velocity: 1.6,
            steering_angle: 0.
        };
        command.send(target);

    }
}

