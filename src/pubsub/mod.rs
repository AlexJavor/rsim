//#[macro_use]
//extern crate crossbeam_channel;
//extern crate crossbeam_utils;

//trait PubSub {
//
//
//    fn
//}


use std::collections::HashMap;
use std::any::TypeId;
//use crossbeam::channel::{Sender, Receiver};
use std::hash::Hash;
use std::sync::mpsc::{Sender, Receiver};
use std::sync::{Arc, Mutex};
use std::fmt::{Display, Debug};
use std::marker::PhantomData;

type Callback<T> = Box<dyn Fn(T) + Send>;

#[derive(Clone, Debug)]
pub struct TopicDesc<Item> {
    pub topic_name: String,
    pub topic_type: TypeId,
    phantom: PhantomData<Item>
}

impl<Item: 'static> TopicDesc<Item> {
    pub fn new(name: String) -> Self {
        TopicDesc {
            topic_name: name,
            topic_type: TypeId::of::<Item>(),
            phantom: PhantomData
        }
    }
}

type TopicId = String;


#[derive(Clone)]
struct Topic<T> {
    type_id: TypeId,
    writer: Sender<T>,
//    reader: Receiver<T>,
    callbacks: Arc<Mutex<Vec<Callback<T>>>>
}

type PlaceHolder = ();

#[derive(Clone)]
pub struct PubSub {

    channels: HashMap<TopicId, Topic<PlaceHolder>>

}


impl PubSub {
    pub fn init() -> Self {
        PubSub {
            channels: HashMap::new()
        }
    }

    fn create_channel<T: 'static + Debug + Copy + Send>(&mut self, topic: &TopicId) {
        let tid: TypeId = TypeId::of::<T>();
        let (writer, reader) = std::sync::mpsc::channel::<T>(); // crossbeam::channel::unbounded::<T>();

        let channel = Topic {
            type_id: tid,
            writer,
//            reader,
            callbacks: Arc::new(Mutex::new(Vec::new()))
        };
        let cbs = channel.callbacks.clone();
        let channel_erased: Topic<PlaceHolder> = unsafe {
            std::mem::transmute(channel)
        };
        self.channels.insert(topic.clone(), channel_erased);

        let topic = topic.clone();
        std::thread::spawn(move || {
            let callbacks = cbs;
            loop {
                match reader.recv() {
                    Ok(res) => {
                        for cb in callbacks.lock().unwrap().iter() {
                            cb.as_ref()(res);
                        }
                    },
                    Err(e) => break // can only happen if sending end was disconnected, meaning no further message can be received.
                }

            }
        });
    }

    fn get_channel<T: 'static + Debug>(&self, topic: &TopicId) -> Option<&Topic<T>> {
        match self.channels.get(topic) {
            Some(ch) => if ch.type_id == TypeId::of::<T>() {
                let ch2: &Topic<T> = unsafe {
                    std::mem::transmute(ch)
                };
                Some(ch2)
            } else {
                None
            },
            None => None
        }
    }

    fn exists_channel(&self, topic: &TopicId) -> bool {
        self.channels.contains_key(topic)
    }
    fn exists_channel_with_type<T: 'static>(&self, topic: &TopicId) -> bool {
        if let Some(ch) = self.channels.get(topic) {
            ch.type_id == TypeId::of::<T>()
        } else {
            false
        }
    }

    pub fn new_publisher<T: 'static + Copy + Send + Debug>(&mut self, topic: &TopicId) -> Result<Sender<T>, &str> {
        if !self.exists_channel(topic) {
            self.create_channel::<T>(topic);
        }
        assert!(self.exists_channel(topic));
        if self.exists_channel_with_type::<T>(topic) {
            Ok(
                self.get_channel::<T>(topic).expect("unexpected")
                    .writer.clone()
            )
        } else {
            Result::Err("Topic already exists with a different type.")
        }
    }

    pub fn poster<T : 'static + Copy + Send + Debug>(&mut self, desc: &TopicDesc<T>) -> Result<Sender<T>, &str> {
        self.new_publisher::<T>(&desc.topic_name)
    }

    pub fn register_callback<T: 'static + Copy + Send + Debug>(&mut self, topic: &TopicId, callback: Callback<T>) -> Result<(), &str> {
        if !self.exists_channel(topic) {
            self.create_channel::<T>(topic);
        }
        assert!(self.exists_channel(topic));
        if self.exists_channel_with_type::<T>(topic) {
            let ch = self.get_channel::<T>(topic).expect("unexpected");
            let x = &ch.callbacks;
            x.lock().unwrap().push(Box::new(callback));

            Ok(())
        } else {
            Result::Err("Topic already exists with a different type.")
        }
    }

    pub fn add_callback<T: 'static + Copy + Send + Debug>(&mut self, topic: &TopicDesc<T>, callback: Callback<T>) -> Result<(), &str> {
        self.register_callback(&topic.topic_name, callback)
    }

    pub fn last_value_cell<T: 'static + Copy + Send + Debug>(&mut self, topic: &TopicDesc<T>) -> Result<Fluent<T>, &str> {
        let cell = Arc::new(crossbeam::atomic::AtomicCell::new(None));
        let cell_clone = cell.clone();
        self.add_callback(&topic, Box::new(move |t: T| {
            cell_clone.store(Some(t));
        }))?;
        Ok(Fluent {
            value: cell
        })
    }
}

pub struct Fluent<T> {
    value: Arc<crossbeam::atomic::AtomicCell<Option<T>>>
}

impl<T: Clone> Fluent<T> {
    pub fn get(&self) -> Option<T> {
        self.value.clone().take()
    }
}

#[cfg(test)]
mod tests {
    // Note this useful idiom: importing names from outer (for mod tests) scope.
    use super::*;
    use std::time::Duration;

    #[test]
    fn test_add() {
        let input = TopicDesc::new("input".into());
        let output = TopicDesc::new("output".into());

        let mut pubsub = PubSub::init();
        let writer = pubsub.poster(&input).unwrap();


        pubsub.add_callback(&input, Box::new(|i: i32| { println!("from callback: {}", i)}));

        let out2 = output.clone();
        let poster = pubsub.poster(&output).unwrap();

        pubsub.add_callback(&input, Box::new( move|i: i32| {
            poster.send(i + 10);
        }));

        let latest = pubsub.last_value_cell(&output).unwrap();

        std::thread::sleep(Duration::from_millis(10));
        assert_eq!(latest.get(), None);

        writer.send(3);
        std::thread::sleep(Duration::from_millis(10));
        assert_eq!(latest.get(), Some(13));

        writer.send(4);
        writer.send(5);
        std::thread::sleep(Duration::from_millis(10));
        assert_eq!(latest.get(), Some(15));


    }


}