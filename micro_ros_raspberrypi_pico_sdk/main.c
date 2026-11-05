#include <stdio.h>
#include <stdlib.h> // abs()
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <rmw_microros/rmw_microros.h> // Key for transport management
#include "pico_uart_transports.h"

// Zumo 2040 Motor Pins
#define LEFT_PWM_PIN           15
#define LEFT_DIR_PIN           11
#define RIGHT_PWM_PIN          14
#define RIGHT_DIR_PIN          10
#define LEFT                   false
#define RIGHT                  true
#define BASE_SPEED             100
#define MAX_SPEED              6000
#define DESIRED_TURN_THRESHOLD 0

// micro-ROS variables
rcl_subscription_t subscriber;
std_msgs__msg__Float32 msg;
rcl_publisher_t debug_publisher;
std_msgs__msg__Float32 debug_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

//MOTOR Variables
bool last_direction = false;      //Used for storing current direction robot is spinning
int timout_counter = 0;
int ang_vel = 0; // The baseline correction speed;
int error = 0;


// Helper to set motor speeds (-255 to 255)
void set_motors(int left_speed, int right_speed) {
    // Set directions
    gpio_put(LEFT_DIR_PIN, left_speed < 0 ? 1 : 0);
    gpio_put(RIGHT_DIR_PIN, right_speed < 0 ? 1 : 0);

    // Set PWM levels (absolute values)
    pwm_set_gpio_level(LEFT_PWM_PIN, abs(left_speed) * 256); // Scaling for 16-bit wrap
    pwm_set_gpio_level(RIGHT_PWM_PIN, abs(right_speed) * 256);
}


// Subscription Callback
void subscription_callback(const void* msgin) {
    //reset timout counter
    timout_counter = 0;
    
    // 1. Cast the incoming raw data into a Float32 message structure
    const std_msgs__msg__Float32* msg = (const std_msgs__msg__Float32*)msgin;
    
    // 2. Extract the single OpenCV value
    float turn_threshold = msg->data - 320.0f;

    // --- Echo the value back for debugging ---
    debug_msg.data = turn_threshold;
    rcl_publish(&debug_publisher, &debug_msg, NULL);

    int base_speed = 100; // The max speed is 6000;

    if (turn_threshold > 30) {
        set_motors(105, -(105)); // Turn Right
        last_direction = RIGHT;
    }
    else if (turn_threshold < -30) {
        set_motors(-(105), 80 + 105); // Turn Left
        last_direction = LEFT;
    }
    else {
        set_motors(0, 0);     // Go Straight
    
    }
    // /* The desired turn_threshold should be zero. Calculate the polarity and 
    //    magnitude of the error between the actual turn_threshold and the desired 
    //    turn_threshold. */
    // error = DESIRED_TURN_THRESHOLD - turn_threshold;

    // /* Convert the error into a correction factor to be applied to the angular velocity. */
    // // new angular velocity = error * correction factor (lets just use the base speed) + current angular velocity
    // ang_vel = ((error * BASE_VEL) >> 3) + ang_vel;

    // if (error <= 0) {
    //     set_motors
    // }

}

void setup_hardware() {
    // Initialize standard IO
    stdio_init_all();

    // Initialize Direction Pins
    gpio_init(LEFT_DIR_PIN);
    gpio_set_dir(LEFT_DIR_PIN, GPIO_OUT);
    gpio_init(RIGHT_DIR_PIN);
    gpio_set_dir(RIGHT_DIR_PIN, GPIO_OUT);

    // Initialize PWM Pins
    gpio_set_function(LEFT_PWM_PIN, GPIO_FUNC_PWM);
    gpio_set_function(RIGHT_PWM_PIN, GPIO_FUNC_PWM);

    uint slice_num_left = pwm_gpio_to_slice_num(LEFT_PWM_PIN);
    uint slice_num_right = pwm_gpio_to_slice_num(RIGHT_PWM_PIN);

    // Configure PWM (Simple 16-bit wrap)
    pwm_set_wrap(slice_num_left, 65535);
    pwm_set_wrap(slice_num_right, 65535);
    pwm_set_enabled(slice_num_left, true);
    pwm_set_enabled(slice_num_right, true);
}

int main() {
    setup_hardware();

    rcl_ret_t rc;

    // This handles the transport internally based on your CMake settings
    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );

    allocator = rcl_get_default_allocator();

    // Wait for micro-ROS Agent to connect over USB
    while (rmw_uros_ping_agent(1000, 1) != RCL_RET_OK) {
        sleep_ms(500);
    }

    // 1. Initialize micro-ROS support & Node FIRST
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "zumo_subscriber_node", "", &support);

    // 2. Initialize Subscriber
    rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),    // Message Type
        "/cv_coords_filtered"                                            // Topic Name
    );

    // 3. Initialize Publisher NOW (since the node is finally ready)
    rc = rclc_publisher_init_default(
        &debug_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/zumo_debug_echo"
    );

    if (rc != RCL_RET_OK) {
        // Turn on LED to signal an error
        gpio_init(PICO_DEFAULT_LED_PIN);
        gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
    }

    // 4. Create executor
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);

    // 5. Spin loop
    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        sleep_ms(1);
        timout_counter++;
        //check for lost robot case
        if ((timout_counter > 9) && (last_direction == RIGHT)){
          set_motors(-100, 100);
          //last_direction = LEFT;
          timout_counter = 0;
        }

        else if((timout_counter > 9) && (last_direction == LEFT)){
          set_motors(100, -100);
          //last_direction = RIGHT;
          timout_counter = 0;
        }
        
    }

    return 0;
}