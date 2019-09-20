(define (problem task)
(:domain hazarddetection)
(:objects
    door_kitchen door_room lamp_room dishwasher_kitchen - hazard
    rico - robot
    initial - robot-location
    door_kitchen_location door_room_location lamp_room_location dishwasher_kitchen_location wp0 wp1 wp2 wp3 - hazard-location
    lidar rgbd ultrasonic - robot-sensor
    door_sensor window_sensor - home-system-sensor
)

(:init
    (at rico initial)
    (not_checking)

    (linked lamp_room wp1)
    (linked door_room wp2)
    (linked door_kitchen wp3)
    (linked dishwasher_kitchen wp0)

    (sensor_type door_kitchen lidar)
    (sensor_type door_room lidar)
    (sensor_type lamp_room rgbd)
    (sensor_type dishwasher_kitchen rgbd)

    ; point which sensors are on robot's platform (1 or 0)
    (= (is_robot_sensor lidar) 1)
    (= (is_robot_sensor rgbd) 1)
    (= (is_robot_sensor ultrasonic) 1)
    (= (is_robot_sensor door_sensor) 0)
    (= (is_robot_sensor window_sensor) 0)

)

(:goal (and
    (checked_light lamp_room)
    (checked_door door_kitchen)
    (checked_door door_room)
    (checked_dishwasher dishwasher_kitchen)
    )
)

)
