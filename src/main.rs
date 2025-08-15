#![no_main]
#![no_std]

use vexide::prelude::*;
use core::time::Duration;

const MAX_MOTOR_POWER: f64 = 12.0;


// Particle struct for MCL
struct Particle {
    x: f64,
    y: f64,
    weight: f64,
}

impl Particle {
    fn new(x: f64, y: f64) -> Self {
        Self { x, y, weight: 1.0 }
    }
}

// Field dimensions (example, adjust as needed)
const FIELD_WIDTH: f64 = 365.76; // cm
const FIELD_HEIGHT: f64 = 365.76; // cm
const NUM_PARTICLES: usize = 10000;

impl Robot {
    // Monte Carlo Localization using distance sensors
    async fn localize_mcl(&mut self) -> (f64, f64, f64) {
        // Simple no_std PRNG (LCG)
        struct NoStdRng {
            state: u32,
        }
        impl NoStdRng {
            fn new(seed: u32) -> Self { Self { state: seed } }
            fn next_u32(&mut self) -> u32 {
                self.state = self.state.wrapping_mul(1664525).wrapping_add(1013904223);
                self.state
            }
            fn next_f64(&mut self) -> f64 {
                (self.next_u32() as f64) / (u32::MAX as f64)
            }
        }

        let mut rng = NoStdRng::new(42); // Fixed seed for repeatability
        let mut particles: [Particle; NUM_PARTICLES] = core::array::from_fn(|_| {
            Particle::new(
                rng.next_f64() * FIELD_WIDTH,
                rng.next_f64() * FIELD_HEIGHT,
            )
        });

        // Read actual sensor values
        let sensor_max_range = 200.0; // cm
        let mut front: f64 = FIELD_HEIGHT.min(sensor_max_range);
        let mut back: f64 = FIELD_HEIGHT.min(sensor_max_range);
        let mut left: f64 = FIELD_WIDTH.min(sensor_max_range);
        let mut right: f64 = FIELD_WIDTH.min(sensor_max_range);

        if let Some(reading) = self.dist_front.object().unwrap_or_default() {
            front = (reading.distance as f64 / 10.0).min(sensor_max_range);
        }
        if let Some(reading) = self.dist_back.object().unwrap_or_default() {
            back = (reading.distance as f64 / 10.0).min(sensor_max_range);
        }
        if let Some(reading) = self.dist_left.object().unwrap_or_default() {
            left = (reading.distance as f64 / 10.0).min(sensor_max_range);
        }
        if let Some(reading) = self.dist_right.object().unwrap_or_default() {
            right = (reading.distance as f64 / 10.0).min(sensor_max_range);
        }

        println!("front = {}, back = {}, left = {}, right = {}", front, back, left, right);

        // Update particle weights based on sensor readings
        for p in particles.iter_mut() {
            // Simulate expected sensor readings for this particle
            let expected_front = FIELD_HEIGHT - p.y;
            let expected_back = p.y;
            let expected_left = p.x;
            let expected_right = FIELD_WIDTH - p.x;

            let sigma = 10.0; // sensor noise (cm)
            let mut weight = 1.0;
            // Only use sensor readings that are in range
            if front < sensor_max_range {
                let w_front = (-((front - expected_front).powi(2)) / (2.0 * sigma * sigma)).exp();
                weight *= w_front;
            }
            if back < sensor_max_range {
                let w_back = (-((back - expected_back).powi(2)) / (2.0 * sigma * sigma)).exp();
                weight *= w_back;
            }
            if left < sensor_max_range {
                let w_left = (-((left - expected_left).powi(2)) / (2.0 * sigma * sigma)).exp();
                weight *= w_left;
            }
            if right < sensor_max_range {
                let w_right = (-((right - expected_right).powi(2)) / (2.0 * sigma * sigma)).exp();
                weight *= w_right;
            }
            p.weight = weight;
        }

        // Normalize weights
        let total_weight: f64 = particles.iter().map(|p| p.weight).sum();
        for p in particles.iter_mut() {
            p.weight /= total_weight.max(1e-12);
        }

        // Resample particles (systematic resampling)
        let mut new_particles: [Particle; NUM_PARTICLES] = core::array::from_fn(|_| Particle::new(0.0, 0.0));
        let mut cumsum = 0.0;
        let mut idx = 0;
        let step = 1.0 / NUM_PARTICLES as f64;
        let mut u = rng.next_f64() * step;
        for i in 0..NUM_PARTICLES {
            while u > cumsum && idx < NUM_PARTICLES {
                cumsum += particles[idx].weight;
                idx += 1;
            }
            if idx > 0 {
                new_particles[i] = Particle::new(particles[idx-1].x, particles[idx-1].y);
            } else {
                new_particles[i] = Particle::new(particles[0].x, particles[0].y);
            }
            u += step;
        }
        particles = new_particles;

        // Estimate pose (weighted mean)
        let mut x_sum = 0.0;
        let mut y_sum = 0.0;
        for p in particles.iter() {
            x_sum += p.x;
            y_sum += p.y;
        }
        let x = x_sum / NUM_PARTICLES as f64;
        let y = y_sum / NUM_PARTICLES as f64;
        let theta = 0.0; // Not estimated here

        (x, y, theta)
    }
}

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
            dist_back: DistanceSensor::new(peripherals.port_19),
            dist_left: DistanceSensor::new(peripherals.port_18),
            dist_right: DistanceSensor::new(peripherals.port_20),
            main_controller: peripherals.primary_controller,
            sub_controller: peripherals.partner_controller,
        }
    }
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        println!("Autonomous! Running MCL...");
        let (x, y, theta) = self.localize_mcl().await;
        println!("Estimated pose: x={:.1}, y={:.1}, theta={:.2}", x, y, theta);
    }

    async fn driver(&mut self) {
            loop {

                // Split arcade drive
                let state = self.main_controller.state().unwrap_or_default();

                let left_y = state.left_stick.y();
                let right_x = state.right_stick.x();

                let left_power = MAX_MOTOR_POWER.min((left_y + right_x) * 12.0);
                let right_power = MAX_MOTOR_POWER.min((left_y - right_x) * 12.0);

                self.left_drive.set_voltage(left_power).ok();
                self.right_drive.set_voltage(right_power).ok();


                // Monte Carlo Localization
                let (x, y, theta) = self.localize_mcl().await;
                println!("Estimated pose: x={:.1}, y={:.1}, theta={:.2}", x, y, theta);


                // Mandatory minimum sleep duration between loops
                sleep(Duration::from_millis(20)).await;
            }
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let mut robot = Robot::new(peripherals);
    // Calibrate IMU and wait for completion
    let _ = robot.imu.calibrate().await;
    robot.compete().await;
}