#![no_main]
#![no_std]

use vexide::prelude::*;
use core::time::Duration;

const MAX_MOTOR_POWER: f64 = 12.0;

// Field dimensions (example, adjust as needed)
const FIELD_WIDTH: f64 = 365.76; // cm
const FIELD_HEIGHT: f64 = 365.76; // cm
const NUM_PARTICLES: usize = 1000;


// PRNG
struct NoStdRng { state: u32 }
impl NoStdRng {
    fn new(seed: u32) -> Self { Self { state: seed } }
    fn next_f64(&mut self) -> f64 {
        self.state = self.state.wrapping_mul(1664525).wrapping_add(1013904223);
        (self.state as f64) / (u32::MAX as f64)
    }
}


#[derive(Clone, Copy)]
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

// Robot struct definition
#[allow(dead_code)]
struct Robot {
    left_drive: Motor,
    right_drive: Motor,
    dr4b_left: Motor,
    dr4b_right: Motor,
    claw: Motor,
    imu: InertialSensor,
    dist_front: DistanceSensor,
    dist_back: DistanceSensor,
    dist_left: DistanceSensor,
    dist_right: DistanceSensor,
    main_controller: Controller,
    sub_controller: Controller,
    prev_left_pos_deg: f64,
    prev_right_pos_deg: f64,
        particles: [Particle; NUM_PARTICLES],
    }

impl Robot {
    // Monte Carlo Localization using distance sensors
    async fn localize_mcl(&mut self) -> (f64, f64, f64) {

        // IMU heading (negated for VEX V5)
        let imu_heading_rad = -self.imu.heading().unwrap_or(0.0).to_radians();
        let directions = [
            (0.0, 1.0),   // front
            (0.0, -1.0),  // back
            (-1.0, 0.0),  // left
            (1.0, 0.0),   // right
        ];

        // Helper: Gaussian probability
        fn gaussian(error: f64, sigma: f64) -> f64 {
            (-error.powi(2) / (2.0 * sigma * sigma)).exp()
        }

        // Helper: Raycast expected sensor reading
        fn expected_distance(x: f64, y: f64, heading: f64, dx: f64, dy: f64) -> f64 {
            let sensor_max_range = 200.0;
            let cos_h = heading.cos();
            let sin_h = heading.sin();
            // Forward = +y, Left = -x, heading=0 means forward
            let dx_r = dx * cos_h + dy * -sin_h;
            let dy_r = dx * sin_h + dy * cos_h;
            let mut dist = sensor_max_range;
            if dx_r.abs() > 1e-6 {
                let tx = if dx_r > 0.0 {
                    (FIELD_WIDTH - x) / dx_r
                } else {
                    -x / dx_r
                };
                if tx > 0.0 && tx < dist {
                    dist = tx;
                }
            }
            if dy_r.abs() > 1e-6 {
                let ty = if dy_r > 0.0 {
                    (FIELD_HEIGHT - y) / dy_r
                } else {
                    -y / dy_r
                };
                if ty > 0.0 && ty < dist {
                    dist = ty;
                }
            }
            dist
        }

    // SAFETY: Only accessed in single-threaded context
    static mut PARTICLES: Option<[Particle; NUM_PARTICLES]> = None;
        let mut rng = NoStdRng::new(42);
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

        // Motion model
        let left_pos_deg = self.left_drive.position().map(|p| p.as_degrees()).unwrap_or(0.0);
        let right_pos_deg = self.right_drive.position().map(|p| p.as_degrees()).unwrap_or(0.0);
        let left_delta = left_pos_deg - self.prev_left_pos_deg;
        let right_delta = right_pos_deg - self.prev_right_pos_deg;
        self.prev_left_pos_deg = left_pos_deg;
        self.prev_right_pos_deg = right_pos_deg;
        let deg_to_cm = 13.2 / 200.0;
        let left_cm = left_delta * deg_to_cm;
        let right_cm = right_delta * deg_to_cm;
        let d_center = (left_cm + right_cm) / 2.0;
        // let d_theta = (right_cm - left_cm) / 18.0;

        for p in particles.iter_mut() {
            let noise_trans = rng.next_f64() * 0.5 - 0.25;
            // let noise_rot = rng.next_f64() * 0.02 - 0.01;
            // Forward = +y, Left = -x, heading=0 means forward
            p.x += (d_center + noise_trans) * -imu_heading_rad.sin();
            p.y += (d_center + noise_trans) * imu_heading_rad.cos();
            p.x = p.x.clamp(0.0, FIELD_WIDTH);
            p.y = p.y.clamp(0.0, FIELD_HEIGHT);
        }

        // Sensor readings
        let sensor_max_range = 200.0;
        let mut sensors = [FIELD_HEIGHT, FIELD_HEIGHT, FIELD_WIDTH, FIELD_WIDTH];
        if let Some(r) = self.dist_front.object().unwrap_or_default() {
            sensors[0] = (r.distance as f64 / 10.0).min(sensor_max_range);
        }
        if let Some(r) = self.dist_back.object().unwrap_or_default() {
            sensors[1] = (r.distance as f64 / 10.0).min(sensor_max_range);
        }
        if let Some(r) = self.dist_left.object().unwrap_or_default() {
            sensors[2] = (r.distance as f64 / 10.0).min(sensor_max_range);
        }
        if let Some(r) = self.dist_right.object().unwrap_or_default() {
            sensors[3] = (r.distance as f64 / 10.0).min(sensor_max_range);
        }

        // Sensor model and weight update
        let sigma = 10.0;
        for p in particles.iter_mut() {
            let mut weight = 1.0;
            for (i, &(dx, dy)) in directions.iter().enumerate() {
                if sensors[i] < sensor_max_range {
                    let expected = expected_distance(p.x, p.y, imu_heading_rad, dx, dy);
                    weight *= gaussian(sensors[i] - expected, sigma);
                }
            }
            p.weight = weight;
        }

        // Normalize weights and reinitialize if collapsed
        let total_weight: f64 = particles.iter().map(|p| p.weight).sum();
        if total_weight < 1e-12 {
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

        // Resampling
    let mut new_particles: [Particle; NUM_PARTICLES] = core::array::from_fn(|_| Particle::new(0.0, 0.0));
        let mut cumsum = 0.0;
        let step = 1.0 / NUM_PARTICLES as f64;
        let mut u = rng.next_f64() * step;
        let mut idx = 0;
        for np in new_particles.iter_mut() {
            while u > cumsum && idx < NUM_PARTICLES {
                cumsum += particles[idx].weight;
                idx += 1;
            }
            if idx > 0 {
                *np = Particle::new(particles[idx-1].x, particles[idx-1].y);
            } else {
                *np = Particle::new(particles[0].x, particles[0].y);
            }
            u += step;
        }
        let identical = new_particles.iter().all(|p| p.x == new_particles[0].x && p.y == new_particles[0].y);
        if identical {
            for p in new_particles.iter_mut() {
                p.x = rng.next_f64() * FIELD_WIDTH;
                p.y = rng.next_f64() * FIELD_HEIGHT;
                p.weight = 1.0;
            }
        }
        for (i, p) in new_particles.iter().enumerate() {
            particles[i] = Particle::new(p.x, p.y);
        }

        // Estimate pose (weighted mean)
        let (mut x_sum, mut y_sum) = (0.0, 0.0);
        for p in particles.iter() {
            x_sum += p.x;
            y_sum += p.y;
        }
        let x = x_sum / NUM_PARTICLES as f64;
        let y = y_sum / NUM_PARTICLES as f64;
        let imu_heading_deg = self.imu.heading().unwrap_or(0.0);
        (x, y, imu_heading_deg)
    }
}

impl Robot {
    fn new(peripherals: Peripherals) -> Self {

        let mut rng = NoStdRng::new(42);

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
            particles: core::array::from_fn(|_| {
                Particle::new(
                    rng.next_f64() * FIELD_WIDTH,
                    rng.next_f64() * FIELD_HEIGHT,
                )
            }),
        }
    }
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        println!("Autonomous! Running MCL...");
        // let particles = &mut self.particles;
        let (x, y, theta) = self.localize_mcl().await;
        println!("Estimated pose: x={:.1}, y={:.1}, theta={:.2}", x, y, theta);


        // Mandatory minimum sleep duration between loops
        sleep(Duration::from_millis(20)).await;
    }

    async fn driver(&mut self) {
        loop {
        // let particles = &mut self.particles;

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
