(define (domain hazarddetection)

(:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions)

(:types
    hazard - linkable
    hazard_door hazard_light hazard_dishwasher - hazard
    robot - locatable
    rico - robot
    hazard-location - location
    robot-sensor home-system-sensor - sensor
)

(:predicates
    (obj_at ?obj - locatable ?loc - location)
    (checked ?hazard - hazard)
    (linked ?hazard - linkable ?loc - location)
    (sensor_type ?haz - hazard ?sen - sensor)
)

(:functions
    (is_automated ?sen - sensor)
)

(:durative-action GO
      :parameters (?obj - robot ?start - location ?destination - location)
      :duration ( = ?duration 30)
      :condition (and
            (at start (obj_at ?obj ?start)))
      :effect (and
            (at end (obj_at ?obj ?destination))
            (at start (not (obj_at ?obj ?start))))
)

(:durative-action CHECK_DOOR
      :parameters (?obj - robot ?hazard - hazard ?hazloc - hazard-location ?sen - sensor)
      :duration ( = ?duration (+ (* (is_automated ?sen) 8) 2))
      :condition (and
            (over all (sensor_type ?hazard ?sen))
            (over all (obj_at ?obj ?hazloc))
            (over all (linked ?hazard ?hazloc)))
      :effect (and
            (at end (checked_door ?hazard)))
)

(:durative-action CHECK_LIGHT
      :parameters (?obj - robot ?hazard - hazard ?hazloc - hazard-location ?sen - sensor)
      :duration ( = ?duration (+ (* (is_automated ?sen) 8) 2))
      :condition (and
            (over all (sensor_type ?hazard ?sen))
            (over all (obj_at ?obj ?hazloc))
            (over all (linked ?hazard ?hazloc)))
      :effect (and
            (at end (checked_light ?hazard)))
)

(:durative-action CHECK_DISHWASHER
      :parameters (?obj - robot ?hazard - hazard ?hazloc - hazard-location ?sen - sensor)
      :duration ( = ?duration (+ (* (is_automated ?sen) 8) 2))
      :condition (and
            (over all (sensor_type ?hazard ?sen))
            (over all (obj_at ?obj ?hazloc))
            (over all (linked ?hazard ?hazloc)))
      :effect (and
            (at end (checked_dishwasher ?hazard)))
)

)