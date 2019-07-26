(define (problem task)
(:domain hazarddetection)
(:objects
    door_kitchen door_room lamp_room dishwasher_kitchen - hazard
    rico - robot
    initial - robot-location
    door_kitchen_location door_room_location lamp_room_location dishwasher_kitchen_location - hazard-location
    lidar rgbd ultrasonic - robot-sensor
    door_sensor window_sensor - home-system-sensor
)

(:init
    (at rico initial)
    (not_checking)

    (linked lamp_room lamp_room_location)
    (linked door_room door_room_location)
    (linked door_kitchen door_kitchen_location)
    (linked dishwasher_kitchen dishwasher_kitchen_location)

    (sensor_type door_kitchen door_sensor)
    (sensor_type door_room door_sensor)
    (sensor_type lamp_room door_sensor)
    (sensor_type dishwasher_kitchen door_sensor)

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
