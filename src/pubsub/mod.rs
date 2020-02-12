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

type Callback<T> = Box<dyn Fn(T) + Send>;

struct SimpleChannel<T> {
    type_id: TypeId,
    writer: Sender<T>,
//    reader: Receiver<T>,
    callbacks: Arc<Mutex<Vec<Callback<T>>>>
}

type PlaceHolder = ();

#[derive(Default)]
struct SimplePubSub<TopicId: Eq + Hash> {

    channels: HashMap<TopicId, SimpleChannel<PlaceHolder>>

}


impl<TopicId> SimplePubSub<TopicId>
where TopicId: Eq + Hash + Clone {
    fn create_channel<T: 'static + Debug + Copy + Send>(&mut self, topic: &TopicId) {
        let tid: TypeId = TypeId::of::<T>();
        let (writer, reader) = std::sync::mpsc::channel::<T>(); // crossbeam::channel::unbounded::<T>();

        let channel = SimpleChannel {
            type_id: tid,
            writer,
//            reader,
            callbacks: Arc::new(Mutex::new(Vec::new()))
        };
        let cbs = channel.callbacks.clone();
        let channel_erased: SimpleChannel<PlaceHolder> = unsafe {
            std::mem::transmute(channel)
        };
        self.channels.insert(topic.clone(), channel_erased);

        std::thread::spawn(move || {
            let callbacks = cbs;
            loop {
                println!("Waiting next value");
                match reader.recv() {
                    Ok(res) => {
                        println!("Got loop: {:?}", res);
                        for cb in callbacks.lock().unwrap().iter() {
                            println!("got cb");
                            cb.as_ref()(res);
                        }
                    },
                    Err(e) => break // can only happen if sending end was disconnected, meaning no further message can be received.
                }

            }
        });
    }

    fn get_channel<T: 'static + Debug>(&self, topic: &TopicId) -> Option<&SimpleChannel<T>> {
        match self.channels.get(topic) {
            Some(ch) => if ch.type_id == TypeId::of::<T>() {
                let ch2: &SimpleChannel<T> = unsafe {
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

    //    pub fn subscribe<T: 'static>(&mut self, topic: &TopicId) -> Result<Receiver<T>, &str> {
//        if !self.exists_channel(topic) {
//            self.create_channel::<T>(topic);
//        }
//        assert!(self.exists_channel(topic));
//        if self.exists_channel_with_type::<T>(topic) {
//                Ok(
//                    self.get_channel::<T>(topic).expect("unexpected")
//                        .reader.clone()
//                )
//
//        } else {
//            Result::Err("Topic already exists with a different type.")
//        }
//    }
    pub fn register_callback<T: 'static + Copy + Send + Debug>(&mut self, topic: &TopicId, callback: Callback<T>) -> Result<(), &str> {
        if !self.exists_channel(topic) {
            self.create_channel::<T>(topic);
        }
        assert!(self.exists_channel(topic));
        if self.exists_channel_with_type::<T>(topic) {
            let ch = self.get_channel::<T>(topic).expect("unexpected");
            let x = ch.callbacks.clone();
            x.lock().unwrap().push(Box::new(callback));

            Ok(())
        } else {
            Result::Err("Topic already exists with a different type.")
        }
    }

    pub fn last_value_cell<T: 'static + Copy + Send + Debug>(&mut self, topic: &TopicId) -> Result<Fluent<T>, &str> {
        let cell = Arc::new(crossbeam::atomic::AtomicCell::new(None));
        let cell_clone = cell.clone();
        self.register_callback(topic, Box::new(move |t: T| {
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

#[cfg(test)]
mod tests {
    // Note this useful idiom: importing names from outer (for mod tests) scope.
    use super::*;
    use std::time::Duration;

    #[test]
    fn test_add() {
        let mut pubsub = SimplePubSub::default();
        let odom_pub = pubsub.new_publisher(&"odom").unwrap();
        odom_pub.send(1);

        std::thread::sleep(Duration::from_secs(1));

        pubsub.register_callback(&"odom", Box::new(|i: i32| { println!("from callback: {}", i)}));


        odom_pub.send(5);

    }


}