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
const NUM_PARTICLES: usize = 1000;

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
        // Static particles buffer, only reinitialize if needed
        static mut PARTICLES: Option<[Particle; NUM_PARTICLES]> = None;
        let particles = unsafe {
            PARTICLES.get_or_insert_with(|| {
                core::array::from_fn(|_| {
                    Particle::new(
                        rng.next_f64() * FIELD_WIDTH,
                        rng.next_f64() * FIELD_HEIGHT,
                    )
                })
            })
        };

        // --- MOTION MODEL: update particles using odometry ---
        let left_pos_deg = self.left_drive.position().map(|p| p.as_degrees()).unwrap_or(0.0);
        let right_pos_deg = self.right_drive.position().map(|p| p.as_degrees()).unwrap_or(0.0);
        let left_delta = left_pos_deg - self.prev_left_pos_deg;
        let right_delta = right_pos_deg - self.prev_right_pos_deg;
        self.prev_left_pos_deg = left_pos_deg;
        self.prev_right_pos_deg = right_pos_deg;

        // Convert degrees to cm (VEX V5 wheel: 200 deg = 13.2 cm for 4" wheel)
        let deg_to_cm = 13.2 / 200.0;
        let left_cm = left_delta * deg_to_cm;
        let right_cm = right_delta * deg_to_cm;
        let d_center = (left_cm + right_cm) / 2.0;
        let d_theta = (right_cm - left_cm) / 18.0; // 18cm track width (adjust as needed)

        // Move each particle
        for p in particles.iter_mut() {
            // Add noise to motion model
            let noise_trans = rng.next_f64() * 0.5 - 0.25; // +/-0.25cm
            let noise_rot = rng.next_f64() * 0.02 - 0.01; // +/-0.01rad
            let theta = self.imu.heading().unwrap_or(0.0).to_radians();
            let dx = (d_center + noise_trans) * theta.cos();
            let dy = (d_center + noise_trans) * theta.sin();
            p.x += dx;
            p.y += dy;
            // Clamp to field
            p.x = p.x.clamp(0.0,FIELD_WIDTH);
            p.y = p.y.clamp(0.0, FIELD_HEIGHT);
        }


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

        // println!("front = {}, back = {}, left = {}, right = {}", front, back, left, right);

        // Get IMU heading in radians
    let imu_heading_deg = self.imu.heading().unwrap_or(0.0);
    let imu_heading_rad = -imu_heading_deg.to_radians();

        // Update particle weights based on sensor readings
        for p in particles.iter_mut() {
            // Rotate sensor directions by IMU heading
            let cos_h = imu_heading_rad.cos();
            let sin_h = imu_heading_rad.sin();

            // Sensor directions in robot frame: front (0,1), back (0,-1), left (-1,0), right (1,0)
            let directions = [
                (0.0, 1.0),   // front
                (0.0, -1.0),  // back
                (-1.0, 0.0),  // left
                (1.0, 0.0),   // right
            ];
            let expected = directions.map(|(dx, dy)| {
                // Rotate by heading
                let dx_r = dx * cos_h - dy * sin_h;
                let dy_r = dx * sin_h + dy * cos_h;
                // Compute distance to wall in rotated direction
                let mut dist = sensor_max_range;
                if dx_r.abs() > 1e-6 {
                    // Intersect with left/right walls
                    let tx = if dx_r > 0.0 {
                        (FIELD_WIDTH - p.x) / dx_r
                    } else {
                        -p.x / dx_r
                    };
                    if tx > 0.0 && tx < dist {
                        dist = tx;
                    }
                }
                if dy_r.abs() > 1e-6 {
                    // Intersect with top/bottom walls
                    let ty = if dy_r > 0.0 {
                        (FIELD_HEIGHT - p.y) / dy_r
                    } else {
                        -p.y / dy_r
                    };
                    if ty > 0.0 && ty < dist {
                        dist = ty;
                    }
                }
                dist
            });

            let sigma = 10.0; // sensor noise (cm)
            let mut weight = 1.0;
            // Only use sensor readings that are in range
            if front < sensor_max_range {
                let w_front = (-((front - expected[0]).powi(2)) / (2.0 * sigma * sigma)).exp();
                weight *= w_front;
            }
            if back < sensor_max_range {
                let w_back = (-((back - expected[1]).powi(2)) / (2.0 * sigma * sigma)).exp();
                weight *= w_back;
            }
            if left < sensor_max_range {
                let w_left = (-((left - expected[2]).powi(2)) / (2.0 * sigma * sigma)).exp();
                weight *= w_left;
            }
            if right < sensor_max_range {
                let w_right = (-((right - expected[3]).powi(2)) / (2.0 * sigma * sigma)).exp();
                weight *= w_right;
            }
            p.weight = weight;
        }

        // Normalize weights
        let total_weight: f64 = particles.iter().map(|p| p.weight).sum();
        if total_weight < 1e-12 {
            // All weights collapsed, reinitialize particles
            for p in particles.iter_mut() {
                p.x = rng.next_f64() * FIELD_WIDTH;
                p.y = rng.next_f64() * FIELD_HEIGHT;
                p.weight = 1.0;
            }
        } else {
            for p in particles.iter_mut() {
                p.weight /= total_weight;
            }
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
        // If all resampled particles are identical, reinitialize
        let identical = new_particles.iter().all(|p| p.x == new_particles[0].x && p.y == new_particles[0].y);
        if identical {
            for p in new_particles.iter_mut() {
                p.x = rng.next_f64() * FIELD_WIDTH;
                p.y = rng.next_f64() * FIELD_HEIGHT;
                p.weight = 1.0;
            }
        }
        // Copy new_particles into static buffer
        unsafe {
            if let Some(pbuf) = PARTICLES.as_mut() {
                for (i, p) in new_particles.iter().enumerate() {
                    pbuf[i] = Particle::new(p.x, p.y);
                }
            }
        }

        // Estimate pose (weighted mean)
        let mut x_sum = 0.0;
        let mut y_sum = 0.0;
        for p in particles.iter() {
            x_sum += p.x;
            y_sum += p.y;
        }
        let x = x_sum / NUM_PARTICLES as f64;
        let y = y_sum / NUM_PARTICLES as f64;
        let theta = self.imu.heading().unwrap_or(0.0);

        (x, y, theta)
    }
}

struct Robot {
    
    // Drivetrain motors
    left_drive: Motor,
    right_drive: Motor,
    // Odometry state
    prev_left_pos_deg: f64,
    prev_right_pos_deg: f64,

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
            prev_left_pos_deg: 0.0,
            prev_right_pos_deg: 0.0,
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