#[macro_use] extern crate lazy_static;
extern crate linkbot_core as lc;
#[macro_use] extern crate log;
extern crate websocket as ws;

use std::env;
use std::thread;
use std::sync::{Arc, Condvar, Mutex, mpsc};
use std::sync::mpsc::sync_channel;
use std::time::Duration;

use ws::{Message};
use ws::client::ClientBuilder;

lazy_static! {
    static ref DAEMON: Mutex<lc::DaemonProxy> = {
        let d = Mutex::new(lc::DaemonProxy::new());
        init_daemon(&d);
        d
    };
}

const DEFAULT_DAEMON_HOST: &'static str = "ws://127.0.0.1:42000";

fn init_daemon(daemon: &Mutex<lc::DaemonProxy>) {
    // Create the websocket connection
    let uri = match env::var("LINKBOT_DAEMON_HOSTPORT") {
        Ok(address) => address,
        _ => String::from(DEFAULT_DAEMON_HOST)
    };
    let client = ClientBuilder::new(uri.as_str())
        .unwrap()
        .connect_insecure()
        .unwrap();
    let (mut ws_rx, mut ws_tx) = client.split().unwrap();
    //let (daemon_write_tx, daemon_write_rx) = sync_channel::<Vec<u8>>(1);
    // Set the daemon proxy's write callback
    {
        info!("Locking daemon in init_daemon()...");
        match daemon.try_lock() {
            Ok(mut d) => {
                info!("Daemon locked in init_daemon()...");
                d.set_write_callback(move |data| {
                    let message = Message::binary(data);
                    ws_tx.send_message(&message).unwrap();
                });
                info!("Daemon unlocked in init_daemon()...");
            } 
            _ => {
                panic!("Could not lock daemon lock!");
            }
        }
        /*
        let mut d = DAEMON.lock().unwrap();
        d.set_write_callback(move |data| {
            let message = Message::binary(data);
            ws_tx.send_message(&message).unwrap();
        });
        */
    }

    // Start the read pump
    thread::spawn( move || {
        info!("Read pump waiting for message...");
        for message_option in ws_rx.incoming_messages() {
            info!("Read pump waiting for message...Received message.");
            if message_option.is_err() {
                warn!("WS connection closed.");
                return;
            }
            let message = message_option.unwrap();
            {
                info!("Locking daemon...");
                let mut d = DAEMON.lock().unwrap();
                match message {
                    ws::OwnedMessage::Binary(data) => d.deliver(&data).unwrap(),
                    _ => warn!("Unexpected websocket data received.")
                }
                info!("Unlocking daemon...");
            }
        }
    });
}

pub struct Linkbot {
    inner: lc::Robot,
    timeout: Duration,
}

impl Linkbot {
    pub fn new(serial_id: &str) -> Linkbot {
        let pair = Arc::new( ( Mutex::new(false), Condvar::new() ) );
        let pair2 = pair.clone();
        let global_daemon = &DAEMON;
        let robot = match global_daemon.try_lock() {
            Ok(mut daemon) => {
                let mut inner = daemon.get_robot(serial_id);
                info!("Setting connect event handler...");
                inner.set_connect_event_handler(move |_| {
                    info!("Robot connect event handler.");
                    let &(ref lock, ref cvar) = &*pair2;
                    let mut started = lock.lock().unwrap();
                    *started = true;
                    info!("Robot connect event handler notifying condvar...");
                    cvar.notify_all();
                });
                info!("Setting connect event handler...done");
                // Send the connect signal
                info!("Sending connect signal...");
                daemon.connect_robot(serial_id);
                info!("Sending connect signal...done");
                Linkbot{ inner: inner,
                         timeout: Duration::from_secs(8),
                }
            }
            _ => {
                panic!("Could not lock daemon!");
            }
        };

        // Wait for the connection event to arrive
        let &(ref lock, ref cvar) = &*pair;
        let mut started = lock.lock().unwrap();
        info!("Waiting for robot connectEvent...");
        while !*started {
            started = cvar.wait(started).unwrap();
        }
        info!("Waiting for robot connectEvent...done");
        robot
        /*
        let mut daemon = global_daemon.lock().unwrap();
        let inner = daemon.get_robot(serial_id);
        Linkbot{inner: inner}
        */
    }

    pub fn get_accelerometer_data(&mut self) -> Result<(f32, f32, f32), String> {
        let (tx, rx) = mpsc::channel::<(f32, f32, f32)>();
        self.inner.get_accelerometer_data(move |x, y, z| {
            tx.send((x, y, z)).unwrap();
        });
        rx.recv_timeout(self.timeout).map_err(|e| { format!("{}", e) } )
    }

    pub fn set_led_color(&mut self, red: u8, green: u8, blue: u8) -> Result<(), String> {
        let (tx, rx) = mpsc::channel::<()>();
        self.inner.set_led_color(red, green, blue, move || {
            tx.send(()).unwrap();
        });
        rx.recv_timeout(self.timeout).map_err(|e| { format!("{}", e) } )
    }

    pub fn enable_button_event(&mut self, handler: Option<Box<lc::ButtonEventHandler>> ) -> Result<(), String> 
    {
        let (tx, rx) = mpsc::channel::<()>();
        let mut enable = false;
        if let Some(mut cb) = handler {
            enable = true;
            self.inner.set_button_event_handler(move |timestamp, button, state| {
                cb(timestamp, button as u32, state as u32);
            });
        }
        self.inner.enable_button_event(enable, move || {
            tx.send(()).unwrap();
        }).unwrap();
        rx.recv_timeout(self.timeout).map_err(|e| { format!("{}", e) } )
    }

}

/*
struct Inner {
    serial_id: String,
    to_daemon: mpsc::SyncSender<Vec<u8>>,
}

impl Inner {
    fn new(serial_id: &str) -> Inner {
        let (sender, receiver) = sync_channel();
        thread::spawn( move || {

        });
    }
}
*/

#[cfg(test)]
mod tests {
    extern crate colored_logger;
    use super::Linkbot;
    #[test]
    fn it_works() {
        colored_logger::init().unwrap();
        use std::io;
        let mut input = String::new();
        println!("Enter a robot's serial ID: ");
        if let Ok(n) = io::stdin().read_line(&mut input) {
            println!("Read serial-id, {} bytes: {}", n, input);
            input.truncate(4);
            let mut l = Linkbot::new(input.as_str());

            println!("Getting accelerometer data...");
            let acceldata = l.get_accelerometer_data().unwrap();
            println!("Accel data: {} {} {}", acceldata.0, acceldata.1, acceldata.2);
            
            println!("Setting LED color...");
            l.set_led_color(255, 255, 255).unwrap();

            println!("Setting robot button handler. Press 'Enter' to continue.");
            let button_handler = |timestamp, button, state| {
                println!("Button press! {}, {}, {}", timestamp, button, state);
            };
            l.enable_button_event(Some( Box::new( button_handler ) ) ).unwrap();
            io::stdin().read_line(&mut input);
            println!("Test complete.");
        }
        
    }
}
