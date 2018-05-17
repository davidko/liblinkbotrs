
extern crate linkbot_core as lc;
extern crate websocket as ws;

use std::clone::Clone;
use std::f32::consts::PI;
use std::sync::{Arc, Condvar, Mutex, mpsc};
use std::time::Duration;

use super::JointStateCommand;
use util;

pub type Result<T> = ::std::result::Result<T, String>;

pub struct Linkbot {
    inner: lc::Robot,
    timeout: Duration,
    joints_moving: Arc< ( Mutex<u8>, Condvar ) >, // mask
    motor_mask: u8, // 0x05 for Linkbot-I, 0x03 for Linkbot-L
}

impl Linkbot {
    pub fn new(serial_id: &str) -> Result<Linkbot> {
        let pair = Arc::new( ( Mutex::new(false), Condvar::new() ) );
        let pair2 = pair.clone();
        let global_daemon = &super::DAEMON;
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
            let result = cvar.wait_timeout(started, robot.timeout).unwrap();
            if result.1.timed_out() {
                return Err(format!("Timed out trying to connect to robot: {}.", serial_id));
            }
            started = result.0;
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

        Ok(robot)
    }

    pub fn acquire() -> Result<Linkbot> {
        // Step 1: Get the serial ID of the first acquirable robot.
        let serial_id_result = {
            let global_daemon = &super::DAEMON;
            match global_daemon.try_lock() {
                Ok(mut daemon) => {
                    let (tx, rx) = mpsc::channel::<Option<String>>();
                    debug!("Acquiring robot from daemon...");
                    daemon.acquire_robot(move |maybe_string| {
                        debug!("Acquired robot: {:?}", maybe_string);
                        tx.send(maybe_string).unwrap();
                    }).unwrap();
                    drop(daemon);
                    rx.recv_timeout(Duration::from_millis(1000))
                },
                _ => {
                    panic!("Could not lock daemon!");
                }
            }
        };
        {
            debug!("acquire result: {:?}", serial_id_result);
            if let Ok(Some(serial_id)) = serial_id_result {
                Linkbot::new(serial_id.as_str())
            } else {
                Err(format!("Could not acquire a Linkbot. Not enough robots in the Robot Manager?"))
            }
        }
    }

    pub fn get_accelerometer_data(&mut self) -> Result<(f32, f32, f32)> {
        let (tx, rx) = mpsc::channel::<(f32, f32, f32)>();
        self.inner.get_accelerometer_data(move |x, y, z| {
            tx.send((x, y, z)).unwrap();
        }).unwrap();
        rx.recv_timeout(self.timeout).map_err(|e| { format!("{}", e) } )
    }

    pub fn get_firmware_version_string(&mut self) -> Result<String> {
        let (tx, rx) = mpsc::channel::<String>();
        self.inner.get_firmware_version_string(move |version| {
            tx.send(version).unwrap();
        }).unwrap();
        rx.recv_timeout(self.timeout).map_err(|e| { format!("{}", e) } )
    }

    pub fn get_form_factor(&mut self) -> Result<lc::FormFactor> {
        let (tx, rx) = mpsc::channel::<lc::FormFactor>();
        self.inner.get_form_factor(move |f| {
            tx.send(f).unwrap();
        }).unwrap();
        rx.recv_timeout(self.timeout).map_err(|e| { format!("{}", e) } )
    }

    pub fn get_joint_angles(&mut self) -> Result<(f32, f32, f32)> {
        let (tx, rx) = mpsc::channel::<(f32, f32, f32)>();
        self.inner.get_encoder_values(move |_, angles| {
            let degrees = angles.iter().map(|&x| x*180.0/PI).collect::<Vec<_>>();
            tx.send((degrees[0], degrees[1], degrees[2])).unwrap();
        }).unwrap();
        rx.recv_timeout(self.timeout).map_err(|e| { format!("{}", e) } )
    }

    pub fn get_joint_states(&mut self) -> 
        Result<(u32, lc::JointState, lc::JointState, lc::JointState)> 
    {
        let (tx, rx) = mpsc::channel::<(u32, lc::JointState, lc::JointState, lc::JointState)>();
        self.inner.get_joint_states(move |timestamp, states| {
            tx.send((timestamp, states[0], states[1], states[2])).unwrap();
        }).unwrap();
        rx.recv_timeout(self.timeout).map_err(|e| { format!("{}", e) } )
    }

    pub fn get_led_color(&mut self) -> Result<(u8, u8, u8)> {
        let (tx, rx) = mpsc::channel::<(u8, u8, u8)>();
        self.inner.get_led_color(move |r, g, b| {
            tx.send((r, g, b)).unwrap();
        }).unwrap();
        rx.recv_timeout(self.timeout).map_err(|e| { format!("{}", e) } )
    }

    pub fn get_joint_speeds(&mut self) -> Result<(f32, f32, f32)> {
        let (tx, rx) = mpsc::channel::<(f32, f32, f32)>();
        self.inner.get_motor_controller_omega(move |omegas| {
            let degrees = omegas.iter().map(|&x| x*180.0/PI).collect::<Vec<_>>();
            tx.send((degrees[0], degrees[1], degrees[2])).unwrap();
        }).unwrap();
        rx.recv_timeout(self.timeout).map_err(|e| { format!("{}", e) } )
    }

    pub fn move_goal<F>(&mut self, mut goals: Vec<Option<lc::Goal>>, cb: F ) -> Result<()> 
        where F: FnMut(),
              F: 'static
    {
        while goals.len() < 3 {
            goals.push(None);
        }
    
        let _goals:Vec<_> = goals.iter().map(|maybe_goal| {
            if let Some(ref goal) = *maybe_goal {
                let value = goal.get_goal();
                let mut _goal = goal.clone();
                _goal.set_goal(value * PI / 180.0);
                Some(_goal)
            } else {
                None
            }
        }).collect();

        let goals_mask = util::vec_to_mask(&_goals);
        self.set_joints_moving(goals_mask);
        self.inner.robot_move(_goals[0].clone(), _goals[1].clone(), _goals[2].clone(), cb)
    }

    pub fn move_goals(&mut self, goals: Vec<Option<lc::Goal>> ) -> Result<()>
    {
        let (tx, rx) = mpsc::channel::<()>();
        self.move_goal(goals, move || {
            tx.send(()).unwrap();
        }).unwrap();
        rx.recv_timeout(self.timeout).map_err(|e| {format!("{}", e)})
    }

    pub fn move_accel(&mut self,
                      states: Vec<Option<(bool, f32, f32, JointStateCommand)>>)
                      -> Result<()>
    {
        //! Each "state" tuple is (relative, omega_0, timeout, endstate)
        let mut goals: Vec<Option<lc::Goal>> = Vec::new();

        for (_i, item) in states.iter().enumerate() {
            // FIXME : Should _endstate be used?
            let maybe_goal = if let &Some((relative, omega, timeout, _endstate)) = item {
                let mut goal = lc::Goal::new();
                goal.set_field_type( 
                    if relative {
                        lc::Goal_Type::RELATIVE
                    } else {
                        lc::Goal_Type::ABSOLUTE
                    });
                goal.set_controller( lc::Goal_Controller::ACCEL );
                goal.set_timeout(timeout as f32);
                goal.set_goal(omega as f32);
                Some(goal)
            } else {
                None
            };
            goals.push(maybe_goal)
        }

        let (tx, rx) = mpsc::channel::<()>();
        self.move_goal(goals, move || {
            tx.send(()).unwrap();
        }).unwrap();
        rx.recv_timeout(self.timeout).map_err(|e| { format!("{}", e) } )
    }

    pub fn move_motors(&mut self, angles: Vec< Option<f32> >) -> Result<()> {
        let goals:Vec<_> = angles.iter().map( |maybe_angle| {
            match *maybe_angle {
                None => None,
                Some(angle) => {
                    let mut goal = lc::Goal::new();
                    goal.set_field_type( lc::Goal_Type::RELATIVE );
                    goal.set_controller( lc::Goal_Controller::CONSTVEL );
                    goal.set_goal(angle);
                    Some(goal)
                }
            }
        }).collect();

        // Send the message
        let (tx, rx) = mpsc::channel::<()>();
        self.move_goal(goals, move || {
            tx.send(()).unwrap();
        }).unwrap();
        rx.recv_timeout(self.timeout).map_err(|e| { format!("{}", e) } )
    }

    pub fn move_motors_to(&mut self, angles: Vec< Option<f32> >) -> Result<()> {
        let goals:Vec<_> = angles.iter().map( |maybe_angle| {
            match *maybe_angle {
                None => None,
                Some(angle) => {
                    let mut goal = lc::Goal::new();
                    goal.set_field_type( lc::Goal_Type::ABSOLUTE );
                    goal.set_controller( lc::Goal_Controller::CONSTVEL );
                    goal.set_goal(angle);
                    Some(goal)
                }
            }
        }).collect();

        // Send the message
        let (tx, rx) = mpsc::channel::<()>();
        self.move_goal(goals, move || {
            tx.send(()).unwrap();
        }).unwrap();
        rx.recv_timeout(self.timeout).map_err(|e| { format!("{}", e) } )
    }

    pub fn move_smooth(&mut self, angles: Vec<Option<(bool, f32)>>) -> Result<()> {
        let goals:Vec<_> = angles.iter().map( |maybe_tuple| {
            if let Some((relative, angle)) = *maybe_tuple {
                let mut goal = lc::Goal::new();
                goal.set_field_type( if relative { lc::Goal_Type::RELATIVE } else { lc::Goal_Type::ABSOLUTE } );
                goal.set_controller( lc::Goal_Controller::SMOOTH );
                goal.set_goal(angle);
                Some(goal)
            } else {
                None
            }
        }).collect();

        // Send the message
        let (tx, rx) = mpsc::channel::<()>();
        self.move_goal(goals, move || {
            tx.send(()).unwrap();
        }).unwrap();
        rx.recv_timeout(self.timeout).map_err(|e| { format!("{}", e) } )
    }

    pub fn move_wait(&mut self, mask: u8) -> Result<()> {
        //! The mask indicates which motors to wait for.

        // Lets get the joint states first.
        let (_, state1, state2, state3) = self.get_joint_states().unwrap();
        if !vec![state1, state2, state3].contains(&lc::JointState::MOVING) {
            return Ok(())
        }
        let pair = self.joints_moving.clone();
        let &(ref lock, ref cvar) = &*pair;
        let mut joints_mask = lock.lock().unwrap();
        while (*joints_mask & mask & self.motor_mask) != 0 {
            joints_mask = cvar.wait(joints_mask).unwrap();
        }
        Ok(())
    }

    pub fn reset_encoders(&mut self) -> Result<()> {
        let (tx, rx) = mpsc::channel::<()>();
        self.inner.reset_encoder_revs(move || {
            tx.send(()).unwrap();
        }).unwrap();
        rx.recv_timeout(self.timeout).map_err(|e| { format!("{}", e) } )
    }

    pub fn reset_to_zero(&mut self) -> Result<()> {
        self.reset_encoders().and_then(|_| {
            self.move_motors_to(vec![Some(0.0), Some(0.0), Some(0.0)])
        })
    }

    pub fn set_alpha_i(&mut self, mask: u32, values: Vec<f32>) -> Result<()> {
        let (tx, rx) = mpsc::channel::<()>();
        let degrees = values.iter().map(|x| x*PI/180.0).collect();
        self.inner.set_motor_controller_alpha_i(mask, degrees, move || {
            tx.send(()).unwrap();
        }).unwrap();
        rx.recv_timeout(self.timeout).map_err(|e| { format!("{}", e) } )
    }

    pub fn set_alpha_f(&mut self, mask: u32, values: Vec<f32>) -> Result<()> {
        let (tx, rx) = mpsc::channel::<()>();
        let degrees = values.iter().map(|x| x*PI/180.0).collect();
        self.inner.set_motor_controller_alpha_f(mask, degrees, move || {
            tx.send(()).unwrap();
        }).unwrap();
        rx.recv_timeout(self.timeout).map_err(|e| { format!("{}", e) } )
    }

    pub fn set_buzzer_frequency(&mut self, frequency: f32) -> Result<()> {
        let (tx, rx) = mpsc::channel::<()>();
        self.inner.set_buzzer_frequency(frequency, move || {
            tx.send(()).unwrap();
        }).unwrap();
        rx.recv_timeout(self.timeout).map_err(|e| { format!("{}", e) } )
    }

    pub fn set_joint_speeds(&mut self, 
                            mask: u32, 
                            speed1: f32, 
                            speed2: f32, 
                            speed3: f32) -> Result<()> {
        let degrees = vec![speed1, speed2, speed3].iter().map(|s| {
            s*PI/180.0
        }).collect();
        let (tx, rx) = mpsc::channel::<()>();
        self.inner.set_motor_controller_omega(
            mask,
            degrees, 
            move || {
                tx.send(()).unwrap();
            }).unwrap();
        rx.recv_timeout(self.timeout).map_err(|e| { format!("{}", e) } )
    }

    pub fn set_joint_states(&mut self, 
                            states: &Vec<Option< (JointStateCommand, f32, Option<f32>, Option<JointStateCommand>) >>)
        -> Result<()>
    {
        //! Each state should have the following information:
        //! (new_state, coefficient, timeout, end_state)
        //let states = vec![state1, state2, state3];

        let joint_state_command_to_proto = |jsc: &JointStateCommand| {
            match *jsc {
                JointStateCommand::Coast => lc::JointState::COAST,
                JointStateCommand::Hold => lc::JointState::HOLD,
                JointStateCommand::Moving => lc::JointState::MOVING,
                _ => lc::JointState::COAST,
            }
        };

        let mut goals:Vec<Option<lc::Goal>> = states.iter().map(|maybe_state| {
            match *maybe_state {
                None => None,
                Some( (ref command, coef, maybe_timeout, ref maybe_end ) ) => {
                    let mut g = lc::Goal::new();
                    match *command {
                        JointStateCommand::Coast => {
                            g.set_field_type( lc::Goal_Type::INFINITE );
                            g.set_controller( lc::Goal_Controller::PID );
                        }
                        JointStateCommand::Hold => {
                            g.set_field_type( lc::Goal_Type::RELATIVE );
                            g.set_controller( lc::Goal_Controller::PID );
                        }
                        JointStateCommand::Power => {
                            g.set_field_type( lc::Goal_Type::INFINITE );
                            g.set_controller( lc::Goal_Controller::CONSTVEL );
                            g.set_goal( coef );
                        }
                        _ => { return None; }
                    }

                    if let Some(timeout) = maybe_timeout {
                        g.set_timeout(timeout);
                    }

                    if let Some(ref end) = *maybe_end {
                        g.set_modeOnTimeout( joint_state_command_to_proto(end) );
                    }
                    Some(g)
                }
            }
        }).collect();
        while goals.len() < 3 {
            goals.push(None);
        }

        let (tx, rx) = mpsc::channel::<()>();
        self.inner.robot_move(goals[0].clone(), goals[1].clone(), goals[2].clone(), move || {
            tx.send(()).unwrap();
        }).unwrap();
        rx.recv_timeout(self.timeout).map_err(|e| { format!("{}", e) } )
    }

    pub fn set_led_color(&mut self, red: u8, green: u8, blue: u8) -> Result<()> {
        let (tx, rx) = mpsc::channel::<()>();
        self.inner.set_led_color(red, green, blue, move || {
            tx.send(()).unwrap();
        }).unwrap();
        rx.recv_timeout(self.timeout).map_err(|e| { format!("{}", e) } )
    }

    pub fn set_reset_on_disconnect(&mut self, mask: u32, peripheral_mask: u32) -> Result<()> {
        let (tx, rx) = mpsc::channel::<()>();
        self.inner.set_reset_on_disconnect(mask, peripheral_mask, move || {
            tx.send(()).unwrap();
        }).unwrap();
        rx.recv_timeout(self.timeout).map_err(|e| { format!("{}", e) } )
    }

    pub fn stop(&mut self, mask: u32) -> Result<()> {
        let (tx, rx) = mpsc::channel::<()>();
        self.inner.stop(Some(mask), move || {
            tx.send(()).unwrap();
        }).unwrap();
        rx.recv_timeout(self.timeout).map_err(|e| { format!("{}", e) } )
    }

    // Callback functions
    
    pub fn enable_accelerometer_event(&mut self, handler: Option<Box<lc::AccelerometerEventHandler>> ) -> Result<()> 
    {
        let (tx, rx) = mpsc::channel::<()>();
        let mut enable = false;
        if let Some(mut cb) = handler {
            enable = true;
            self.inner.set_accelerometer_event_handler(move |timestamp, x, y, z| {
                cb(timestamp, x, y, z);
            });
        }
        self.inner.enable_accelerometer_event(enable, None, move || {
            tx.send(()).unwrap();
        }).unwrap();
        rx.recv_timeout(self.timeout).map_err(|e| { format!("{}", e) } )
    }
    

    pub fn enable_button_event(&mut self, handler: Option<Box<lc::ButtonEventHandler>> ) -> Result<()> 
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

    pub fn enable_encoder_event(&mut self, handler: Option<Box<lc::EncoderEventHandler>> ) -> Result<()>
    {
        let (tx, rx) = mpsc::channel::<()>();
        let mut s1 = None;
        let mut s2 = None;
        let mut s3 = None;
        if let Some(mut cb) = handler {
            s1 = Some(2.0);
            s2 = Some(2.0);
            s3 = Some(2.0);
            self.inner.set_encoder_event_handler(move |timestamp, mask, values| {
                cb(timestamp, mask, values);
            });
        }
        self.inner.enable_encoder_event(s1, s2, s3, move || {
            tx.send(()).unwrap();
        }).unwrap();
        rx.recv_timeout(self.timeout).map_err(|e| { format!("{}", e) } )
    }

    // MISC Functions
    
    pub fn write_twi(&mut self, address: u32, data: Vec<u8>) -> Result<()> {
        let (tx, rx) = mpsc::channel::<()>();
        self.inner.write_twi(address, data, move || {
            tx.send(()).unwrap();
        }).unwrap();
        rx.recv_timeout(self.timeout).map_err(|e| { format!("{}", e) } )
    }

    pub fn read_twi(&mut self, address: u32, recvsize: usize) -> Result<Vec<u8>> {
        let (tx, rx) = mpsc::channel::<Vec<u8>>();
        self.inner.read_twi(address, recvsize as u32, move |data| {
            tx.send(data).unwrap();
        }).unwrap();
        rx.recv_timeout(self.timeout).map_err(|e| { format!("{}", e) } )
    }

    pub fn write_read_twi(&mut self, address: u32, recvsize: usize, data: Vec<u8>) -> Result<Vec<u8>> {
        let (tx, rx) = mpsc::channel::<Vec<u8>>();
        self.inner.write_read_twi(address, recvsize as u32, data, move |data| {
            tx.send(data).unwrap();
        }).unwrap();
        rx.recv_timeout(self.timeout).map_err(|e| { format!("{}", e) } )
    }

    pub fn read_eeprom(&mut self, address: u32, size: usize) -> Result<Vec<u8>> {
        let (tx, rx) = mpsc::channel::<Vec<u8>>();
        self.inner.read_eeprom(address, size as u32, move |data| {
            tx.send(data).unwrap();
        }).unwrap();
        rx.recv_timeout(self.timeout).map_err(|e| { format!("{}", e) } )
    }

    pub fn write_eeprom(&mut self, address: u32, data: Vec<u8>) -> Result<()> {
        let (tx, rx) = mpsc::channel::<()>();
        self.inner.write_eeprom(address, data, move || {
            tx.send(()).unwrap();
        }).unwrap();
        rx.recv_timeout(self.timeout).map_err(|e| { format!("{}", e) } )
    }

    fn set_joints_moving(&mut self, mask: u8) {
        let pair = self.joints_moving.clone();
        let &(ref lock, _) = &*pair;
        let mut joints_mask = lock.lock().unwrap();
        *joints_mask |= mask;
    }
}

