(define (problem task)
(:domain hazarddetection)
(:objects
    door_kitchen door_room door_main lamp_room light_kitchen dishwasher_kitchen - hazard
    rico - robot
    door_kitchen_location door_room_location door_main_location lamp_room_location light_kitchen_location dishwasher_kitchen_location wp0 wp1 wp2 wp3 - hazard-location
    initial - robot-location
    lidar rgbd ultrasonic - robot-sensor
    door_sensor window_sensor light_sensor - home-system-sensor
)
(:init
    (at rico initial)




    (linked lamp_room wp1)
    (linked light_kitchen light_kitchen_location)
    (linked door_room wp2)
    (linked door_kitchen wp3)
    (linked door_main door_main_location)
    (linked dishwasher_kitchen wp0)

    (sensor_type door_kitchen lidar)
    (sensor_type door_room lidar)
    (sensor_type door_main door_sensor)
    (sensor_type lamp_room rgbd)
    (sensor_type light_kitchen light_sensor)
    (sensor_type dishwasher_kitchen rgbd)


    (not_checking)

    (= (is_robot_sensor lidar) 1)
    (= (is_robot_sensor rgbd) 1)
    (= (is_robot_sensor ultrasonic) 1)
    (= (is_robot_sensor door_sensor) 0)
    (= (is_robot_sensor window_sensor) 0)
    (= (is_robot_sensor light_sensor) 0)

)
(:goal (and
    (checked_light lamp_room)
    (checked_light light_kitchen)
    (checked_door door_kitchen)
    (checked_door door_room)
    (checked_door door_main)
    (checked_dishwasher dishwasher_kitchen)
))
)
