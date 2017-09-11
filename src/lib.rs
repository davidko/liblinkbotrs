#[macro_use] extern crate lazy_static;
extern crate linkbot_core as lc;
#[macro_use] extern crate log;
extern crate websocket as ws;

use std::env;
use std::f32::consts::PI;
use std::thread;
use std::sync::{Arc, Condvar, Mutex, mpsc};
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
    joints_moving: Arc< ( Mutex<u8>, Condvar ) >, // mask
    motor_mask: u8, // 0x05 for Linkbot-I, 0x03 for Linkbot-L
}

impl Linkbot {
    pub fn new(serial_id: &str) -> Linkbot {
        let pair = Arc::new( ( Mutex::new(false), Condvar::new() ) );
        let pair2 = pair.clone();
        let global_daemon = &DAEMON;
        let mut robot = match global_daemon.try_lock() {
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
                daemon.connect_robot(serial_id).unwrap();
                info!("Sending connect signal...done");
                Linkbot{ inner: inner,
                         timeout: Duration::from_secs(8),
                         joints_moving: Arc::new( ( Mutex::new(0), Condvar::new() ) ),
                         motor_mask: 0,
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

        // Enable the robot joint events
        let joints_moving_pair = robot.joints_moving.clone();
        robot.inner.set_joint_event_handler(move |_, joint, event, _| {
            if (event == lc::JointState::COAST) ||
               (event == lc::JointState::HOLD) {
                   let &(ref lock, ref cvar) = &*joints_moving_pair;
                   let mut mask = lock.lock().unwrap();
                   *mask &= !(1<<joint);
                   cvar.notify_all();
            }
        });
        let (tx, rx) = mpsc::channel::<()>();
        robot.inner.enable_joint_event(true, move|| {
            tx.send(()).unwrap();
        }).unwrap();
        rx.recv_timeout(robot.timeout).unwrap();

        if let Ok(form) = robot.get_form_factor() {
            robot.motor_mask = match form {
                lc::FormFactor::I => 0x05,
                lc::FormFactor::L => 0x03,
                lc::FormFactor::T => 0x07,
                _ => 0x00,
            }
        }

        robot
    }

    pub fn get_accelerometer_data(&mut self) -> Result<(f32, f32, f32), String> {
        let (tx, rx) = mpsc::channel::<(f32, f32, f32)>();
        self.inner.get_accelerometer_data(move |x, y, z| {
            tx.send((x, y, z)).unwrap();
        }).unwrap();
        rx.recv_timeout(self.timeout).map_err(|e| { format!("{}", e) } )
    }

    pub fn get_form_factor(&mut self) -> Result<lc::FormFactor, String> {
        let (tx, rx) = mpsc::channel::<lc::FormFactor>();
        self.inner.get_form_factor(move |f| {
            tx.send(f).unwrap();
        }).unwrap();
        rx.recv_timeout(self.timeout).map_err(|e| { format!("{}", e) } )
    }

    pub fn move_motors(&mut self, 
                mask: u8,
                angle1: f32,
                angle2: f32,
                angle3: f32,
                ) -> Result<(), String> {
        let mut goal_template = lc::Goal::new();
        goal_template.set_field_type( lc::Goal_Type::RELATIVE );
        goal_template.set_controller( lc::Goal_Controller::CONSTVEL );

        let angles = vec![angle1, angle2, angle3];
        let mut goals = Vec::new();
        for i in 0..3 {
            let g = if (mask & (1<<i)) != 0 {
                let mut g = goal_template.clone();
                g.set_goal(angles[i]*PI/180.0);
                Some(g)
            } else {
                None
            };
            goals.push(g);
        }

        // Set our "joints moving" indicator
        {
            let pair = self.joints_moving.clone();
            let &(ref lock, _) = &*pair;
            let mut joints_mask = lock.lock().unwrap();
            *joints_mask |= mask;
        }

        // Send the message
        let (tx, rx) = mpsc::channel::<()>();
        self.inner.robot_move(goals[0].clone(), goals[1].clone(), goals[2].clone(), move || {
            tx.send(()).unwrap();
        }).unwrap();
        rx.recv_timeout(self.timeout).map_err(|e| { format!("{}", e) } )
    }

    pub fn move_wait(&mut self, mask: u8) -> Result<(), String> {
        //! The mask indicates which motors to wait for.

        let pair = self.joints_moving.clone();
        let &(ref lock, ref cvar) = &*pair;
        let mut joints_mask = lock.lock().unwrap();
        while (*joints_mask & mask & self.motor_mask) != 0 {
            joints_mask = cvar.wait(joints_mask).unwrap();
        }
        Ok(())
    }

    pub fn set_led_color(&mut self, red: u8, green: u8, blue: u8) -> Result<(), String> {
        let (tx, rx) = mpsc::channel::<()>();
        self.inner.set_led_color(red, green, blue, move || {
            tx.send(()).unwrap();
        }).unwrap();
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

            println!("Testing motors. Moving motors 1, 2, and 3 90, 180, and 360 degrees, respectively...");
            l.move_motors(0x07, 90.0, 180.0, 360.0);
            l.move_wait(0x07);
            println!("Test complete.");
        }
        
    }
}
