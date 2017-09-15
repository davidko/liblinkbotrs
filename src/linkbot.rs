
extern crate linkbot_core as lc;
extern crate websocket as ws;

use std::clone::Clone;
use std::f32::consts::PI;
use std::sync::{Arc, Condvar, Mutex, mpsc};
use std::time::Duration;

use super::JointStateCommand;
use util;


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

    pub fn get_firmware_version_string(&mut self) -> Result<String, String> {
        let (tx, rx) = mpsc::channel::<String>();
        self.inner.get_firmware_version_string(move |version| {
            tx.send(version).unwrap();
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

    pub fn get_joint_angles(&mut self) -> Result<(f32, f32, f32), String> {
        let (tx, rx) = mpsc::channel::<(f32, f32, f32)>();
        self.inner.get_encoder_values(move |_, angles| {
            let degrees = angles.iter().map(|&x| x*180.0/PI).collect::<Vec<_>>();
            tx.send((degrees[0], degrees[1], degrees[2])).unwrap();
        }).unwrap();
        rx.recv_timeout(self.timeout).map_err(|e| { format!("{}", e) } )
    }

    pub fn get_joint_states(&mut self) -> 
        Result<(u32, lc::JointState, lc::JointState, lc::JointState), String> 
    {
        let (tx, rx) = mpsc::channel::<(u32, lc::JointState, lc::JointState, lc::JointState)>();
        self.inner.get_joint_states(move |timestamp, states| {
            tx.send((timestamp, states[0], states[1], states[2])).unwrap();
        }).unwrap();
        rx.recv_timeout(self.timeout).map_err(|e| { format!("{}", e) } )
    }

    pub fn get_led_color(&mut self) -> Result<(u8, u8, u8), String> {
        let (tx, rx) = mpsc::channel::<(u8, u8, u8)>();
        self.inner.get_led_color(move |r, g, b| {
            tx.send((r, g, b)).unwrap();
        }).unwrap();
        rx.recv_timeout(self.timeout).map_err(|e| { format!("{}", e) } )
    }

    pub fn get_joint_speeds(&mut self) -> Result<(f32, f32, f32), String> {
        let (tx, rx) = mpsc::channel::<(f32, f32, f32)>();
        self.inner.get_motor_controller_omega(move |omegas| {
            let degrees = omegas.iter().map(|&x| x*180.0/PI).collect::<Vec<_>>();
            tx.send((degrees[0], degrees[1], degrees[2])).unwrap();
        }).unwrap();
        rx.recv_timeout(self.timeout).map_err(|e| { format!("{}", e) } )
    }

    pub fn move_accel(&mut self,
                      states: Vec<Option<(bool, f32, f32, JointStateCommand)>>)
                      -> Result<(), String>
    {
        //! Each "state" tuple is (relative, omega_0, timeout, endstate)
        let mut goals: Vec<Option<lc::Goal>> = Vec::new();

        for (i, item) in states.iter().enumerate() {
            let maybe_goal = if let &Some((relative, omega, timeout, endstate)) = item {
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

        while goals.len() < 3 {
            goals.push(None);
        }

        let goals_mask = util::vec_to_mask(&goals);
        self.set_joints_moving(goals_mask);


        let (tx, rx) = mpsc::channel::<()>();
        self.inner.robot_move(goals[0].clone(), goals[1].clone(), goals[2].clone(), move || {
            tx.send(()).unwrap();
        }).unwrap();
        rx.recv_timeout(self.timeout).map_err(|e| { format!("{}", e) } )
    }

    pub fn move_motors(&mut self, angles: Vec< Option<f32> >) -> Result<(), String> {
        let mut goals:Vec<_> = angles.iter().map( |maybe_angle| {
            match *maybe_angle {
                None => None,
                Some(angle) => {
                    let mut goal = lc::Goal::new();
                    goal.set_field_type( lc::Goal_Type::RELATIVE );
                    goal.set_controller( lc::Goal_Controller::CONSTVEL );
                    goal.set_goal(angle * PI / 180.0);
                    Some(goal)
                }
            }
        }).collect();

        while goals.len() < 3 {
            goals.push(None);
        }

        self.set_joints_moving( util::vec_to_mask(&goals) );

        // Send the message
        let (tx, rx) = mpsc::channel::<()>();
        self.inner.robot_move(goals[0].clone(), goals[1].clone(), goals[2].clone(), move || {
            tx.send(()).unwrap();
        }).unwrap();
        rx.recv_timeout(self.timeout).map_err(|e| { format!("{}", e) } )
    }

    pub fn move_motors_to(&mut self, angles: Vec< Option<f32> >) -> Result<(), String> {
        let mut goals:Vec<_> = angles.iter().map( |maybe_angle| {
            match *maybe_angle {
                None => None,
                Some(angle) => {
                    let mut goal = lc::Goal::new();
                    goal.set_field_type( lc::Goal_Type::ABSOLUTE );
                    goal.set_controller( lc::Goal_Controller::CONSTVEL );
                    goal.set_goal(angle * PI / 180.0);
                    Some(goal)
                }
            }
        }).collect();

        while goals.len() < 3 {
            goals.push(None);
        }

        self.set_joints_moving( util::vec_to_mask(&goals) );

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

    pub fn reset_encoders(&mut self) -> Result<(), String> {
        let (tx, rx) = mpsc::channel::<()>();
        self.inner.reset_encoder_revs(move || {
            tx.send(()).unwrap();
        }).unwrap();
        rx.recv_timeout(self.timeout).map_err(|e| { format!("{}", e) } )
    }

    pub fn reset_to_zero(&mut self) -> Result<(), String> {
        self.reset_encoders().and_then(|_| {
            self.move_motors_to(vec![Some(0.0), Some(0.0), Some(0.0)])
        })
    }

    pub fn set_alpha_i(&mut self, mask: u32, values: Vec<f32>) -> Result<(), String> {
        let (tx, rx) = mpsc::channel::<()>();
        self.inner.set_motor_controller_alpha_i(mask, values, move || {
            tx.send(()).unwrap();
        }).unwrap();
        rx.recv_timeout(self.timeout).map_err(|e| { format!("{}", e) } )
    }

    pub fn set_alpha_f(&mut self, mask: u32, values: Vec<f32>) -> Result<(), String> {
        let (tx, rx) = mpsc::channel::<()>();
        self.inner.set_motor_controller_alpha_f(mask, values, move || {
            tx.send(()).unwrap();
        }).unwrap();
        rx.recv_timeout(self.timeout).map_err(|e| { format!("{}", e) } )
    }

    pub fn set_buzzer_frequency(&mut self, frequency: f32) -> Result<(), String> {
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
                            speed3: f32) -> Result<(), String> {
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
                            state1: &Option< (JointStateCommand, f32, Option<f32>, Option<JointStateCommand>) >,
                            state2: &Option< (JointStateCommand, f32, Option<f32>, Option<JointStateCommand>) >,
                            state3: &Option< (JointStateCommand, f32, Option<f32>, Option<JointStateCommand>) >)
        -> Result<(), String>
    {
        let states = vec![state1, state2, state3];

        let joint_state_command_to_proto = |jsc: &JointStateCommand| {
            match *jsc {
                JointStateCommand::Coast => lc::JointState::COAST,
                JointStateCommand::Hold => lc::JointState::HOLD,
                JointStateCommand::Moving => lc::JointState::MOVING,
                _ => lc::JointState::COAST,
            }
        };

        let goals:Vec<Option<lc::Goal>> = states.iter().map(|maybe_state| {
            match **maybe_state {
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
        let (tx, rx) = mpsc::channel::<()>();
        self.inner.robot_move(goals[0].clone(), goals[1].clone(), goals[2].clone(), move || {
            tx.send(()).unwrap();
        }).unwrap();
        rx.recv_timeout(self.timeout).map_err(|e| { format!("{}", e) } )
    }


    pub fn set_led_color(&mut self, red: u8, green: u8, blue: u8) -> Result<(), String> {
        let (tx, rx) = mpsc::channel::<()>();
        self.inner.set_led_color(red, green, blue, move || {
            tx.send(()).unwrap();
        }).unwrap();
        rx.recv_timeout(self.timeout).map_err(|e| { format!("{}", e) } )
    }

    pub fn stop(&mut self, mask: u32) -> Result<(), String> {
        let (tx, rx) = mpsc::channel::<()>();
        self.inner.stop(Some(mask), move || {
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

    fn set_joints_moving(&mut self, mask: u8) {
        let pair = self.joints_moving.clone();
        let &(ref lock, _) = &*pair;
        let mut joints_mask = lock.lock().unwrap();
        *joints_mask |= mask;
    }
}

