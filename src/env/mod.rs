use na::{Vector2};
use nphysics2d::object::{DefaultBodySet, DefaultColliderSet, BodyPartHandle, RigidBody, DefaultBodyHandle, DefaultColliderHandle};
use nphysics2d::force_generator::DefaultForceGeneratorSet;
use nphysics2d::joint::DefaultJointConstraintSet;
use nphysics2d::world::{DefaultMechanicalWorld, DefaultGeometricalWorld, MechanicalWorld, GeometricalWorld};





use ncollide2d::shape::{ShapeHandle};
use nphysics2d::object::ColliderDesc;



use crate::robot::{BotDesc};
use crate::pubsub::{PubSub};



pub struct Sensor<World> {
    pub rate: f64,
    pub func: Box<dyn Fn(&World)>
}
pub type Actuator<World>  = Box<dyn Fn(&mut World)>;

pub trait Simulated<World> {
    fn sensors(&self, pubsub: &mut PubSub) -> Vec<Sensor<World>>;
    fn actuators(&self, pubsub: &mut PubSub) -> Vec<Actuator<World>>;
}

pub trait Simu {
    fn step(&mut self);
}

type N = f64;
type BodySet = DefaultBodySet<N>;
type BodyHandle = DefaultBodyHandle;
type ColliderHandle = DefaultColliderHandle;

pub(crate) struct World2D {
    pub mechanical_world: MechanicalWorld<N, BodySet, ColliderHandle>,
    pub geometrical_world: GeometricalWorld<N, BodyHandle, ColliderHandle>,
    pub bodies: BodySet,
    pub colliders: DefaultColliderSet<N>,
    pub joint_constraints: DefaultJointConstraintSet<N, BodySet>,
    pub force_generators: DefaultForceGeneratorSet<N, BodySet>
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

pub(crate) struct Env<World> {
    pub pubsub: PubSub,
    pub world: World,
    pub robots: Vec<BotDesc>,
    pub sensors: Vec<(Sensor<World>, f64)>,
    pub actuators: Vec<Actuator<World>>
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
        for a in bot.actuators(&mut self.pubsub) {
            self.actuators.push(a);
        }
        self.robots.push(bot.clone());
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