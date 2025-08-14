#![no_main]
#![no_std]

use vexide::prelude::*;
use core::time::Duration;

struct Robot {
    
    // Drivetrain motors
    left_drive: Motor,
    right_drive: Motor,

    // DR4B motors
    dr4b_left: Motor,
    dr4b_right: Motor,

    // Claw motor
    claw: Motor,

    // IMU
    imu: InertialSensor,

    // Distance sensors
    dist_front: DistanceSensor,
    dist_back: DistanceSensor,
    dist_left: DistanceSensor,
    dist_right: DistanceSensor,

    // Controllers

    main_controller: Controller,
    sub_controller: Controller,

}

impl Robot {
    fn new(peripherals: Peripherals) -> Self {
        Self {
            left_drive: Motor::new(peripherals.port_1, Gearset::Blue, Direction::Forward),
            right_drive: Motor::new(peripherals.port_2, Gearset::Blue, Direction::Reverse),
            dr4b_left: Motor::new(peripherals.port_8, Gearset::Blue, Direction::Forward),
            dr4b_right: Motor::new(peripherals.port_9, Gearset::Blue, Direction::Forward),
            claw: Motor::new(peripherals.port_10, Gearset::Blue, Direction::Forward),
            imu: InertialSensor::new(peripherals.port_11),
            dist_front: DistanceSensor::new(peripherals.port_17),
            dist_back: DistanceSensor::new(peripherals.port_18),
            dist_left: DistanceSensor::new(peripherals.port_19),
            dist_right: DistanceSensor::new(peripherals.port_20),
            main_controller: peripherals.primary_controller,
            sub_controller: peripherals.partner_controller,
        }
    }
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        println!("Autonomous!");
    }

    async fn driver(&mut self) {
            // Split arcade drive
            loop {

                let state = self.main_controller.state().unwrap_or_default();
                // Main controller (left side)
                let left_y = state.left_stick.y();
                let right_x = state.right_stick.x();

                let left_power = left_y * 12.0;
                let right_power = right_x * 12.0;

                self.left_drive.set_voltage(left_power).ok();
                self.right_drive.set_voltage(right_power).ok();

                sleep(Duration::from_millis(20)).await;
            }
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let robot = Robot::new(peripherals);
    robot.compete().await;
}