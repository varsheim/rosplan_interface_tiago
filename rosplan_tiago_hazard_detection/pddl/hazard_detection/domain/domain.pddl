(define (domain hazarddetection)

(:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions)

(:types
    hazard - linkable
    robot - locatable
    hazard-location - location
    sensor
)

(:predicates
    (obj_at ?obj - locatable ?loc - location )
    (checked ?hazard - hazard)
    (linked ?hazard - linkable ?loc - location)
    (sensor_type ?haz - hazard ?sen - sensor)
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

(:durative-action CHECK
      :parameters (?obj - robot ?hazard - hazard ?hazloc - hazard-location ?sen - sensor)
      :duration ( = ?duration (+ (* (is_robot_sensor ?sen) 8) 2))
      :condition (and
            (over all (sensor_type ?hazard ?sen))
            (over all (obj_at ?obj ?hazloc))
            (over all (linked ?hazard ?hazloc)))
      :effect (and
            (at end (checked ?hazard)))
)




)