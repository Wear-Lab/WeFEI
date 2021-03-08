import pyrealsense2 as rs
import numpy as np
import time
 
def initialize_camera():
	# start the frames pipe
	p = rs.pipeline()
	conf = rs.config()
	conf.enable_stream(rs.stream.accel)
	conf.enable_stream(rs.stream.gyro)
	prof = p.start(conf)
	return p


def gyro_data(gyro):
	return np.asarray([gyro.x, gyro.y, gyro.z])


def accel_data(accel):
	return np.asarray([accel.x, accel.y, accel.z])

p = initialize_camera()
try:
	while True:
		f = p.wait_for_frames()
		accel = accel_data(f[0].as_motion_frame().get_motion_data())
		gyro = gyro_data(f[1].as_motion_frame().get_motion_data())
		print("accelerometer: ", accel)
		print("gyro: ", gyro)
		time.sleep(0.5)
finally:
	p.stop()




'''if (rs2::pose_frame pose_frame = frameset.first_or_default(RS2_STREAM_POSE))
{
rs2_pose pose_sample = pose_frame.get_pose_data();
std::cout << "Pose:" << pose_sample.translation.x << ", " << pose_sample.translation.y << ", " << pose_sample.translation.z << std::endl;
#...
}'''


'''

void process_gyro(rs2_vector gyro_data, double ts)
{
    if (first) // On the first iteration, use only data from accelerometer to set the camera's initial position
    {
        last_ts_gyro = ts;
        return;
    }
    // Holds the change in angle, as calculated from gyro
    float3 gyro_angle;

     // Initialize gyro_angle with data from gyro
    gyro_angle.x = gyro_data.x; // Pitch
    gyro_angle.y = gyro_data.y; // Yaw
    gyro_angle.z = gyro_data.z; // Roll

    // Compute the difference between arrival times of previous and current gyro frames
    double dt_gyro = (ts - last_ts_gyro) / 1000.0;
    last_ts_gyro = ts;

    // Change in angle equals gyro measurements * time passed since last measurement
    gyro_angle = gyro_angle * dt_gyro;

    // Apply the calculated change of angle to the current angle (theta)
    std::lock_guard<std::mutex> lock(theta_mtx);
    theta.add(-gyro_angle.z, -gyro_angle.y, gyro_angle.x);
}

void process_accel(rs2_vector accel_data)
{
    // Holds the angle as calculated from accelerometer data
    float3 accel_angle;
    // Calculate rotation angle from accelerometer data
    accel_angle.z = atan2(accel_data.y, accel_data.z);
    accel_angle.x = atan2(accel_data.x, sqrt(accel_data.y * accel_data.y + accel_data.z * accel_data.z));
    // If it is the first iteration, set initial pose of camera according to accelerometer data
    // (note the different handling for Y axis)
    std::lock_guard<std::mutex> lock(theta_mtx);
    if (first)
    {
        first = false;
        theta = accel_angle;
        // Since we can't infer the angle around Y axis using accelerometer data, we'll use PI as a convetion
        // for the initial pose
        theta.y = PI;
    }
    else
    {
        /* 
        Apply Complementary Filter:
            - "high-pass filter" = theta * alpha:  allows short-duration signals to pass through while
              filtering out signals that are steady over time, is used to cancel out drift.
            - "low-pass filter" = accel * (1- alpha): lets through long term changes, filtering out short
              term fluctuations 
        */
        theta.x = theta.x * alpha + accel_angle.x * (1 - alpha);
        theta.z = theta.z * alpha + accel_angle.z * (1 - alpha);
    }
}

'''
