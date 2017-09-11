#[macro_use] extern crate lazy_static;
extern crate linkbot_core as lc;
#[macro_use] extern crate log;
extern crate websocket as ws;

use std::env;
use std::thread;
use std::sync::{Mutex};

use ws::{Message};
use ws::client::ClientBuilder;

mod linkbot;

pub use linkbot::{Linkbot};

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

            println!("Testing motors. Moving motors 1, 2, and 3 90, 180, and 360 degrees, respectively...");
            l.move_motors(0x07, 90.0, 180.0, 360.0);
            l.move_wait(0x07);
            println!("Test complete.");

            println!("Getting joint angles:");
            if let Ok(angles) = l.get_joint_angles() {
                println!("{} {} {}", angles.0, angles.1, angles.2);
            } else {
                panic!("Could not get joint angles.");
            }

            println!("Getting LED color:");
            if let Ok(color) = l.get_led_color() {
                println!("{} {} {}", color.0, color.1, color.2);
            } else {
                panic!("Could not get led color."); 
            }

            println!("Getting joint speeds:");
            if let Ok(speeds) = l.get_joint_speeds() {
                println!("{} {} {}", speeds.0, speeds.1, speeds.2);
            } else {
                panic!("Could not get joint speeds.");
            }
        }
        
    }
}
