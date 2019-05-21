(define (domain wandering)

(:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions)

(:types
    hazard
    robot - locatable
    hazard-location - location
)

(:predicates
    (obj_at ?obj - locatable ?loc - location )
    (checked ?hazard - hazard)
    (linked ?hazard - hazard ?loc - hazard-location)
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
      :parameters (?obj - robot ?hazard - hazard ?hazloc - hazard-location)
      :duration ( = ?duration 10)
      :condition (and
            (over all (obj_at ?obj ?hazloc))
            (over all (linked ?hazard ?hazloc)))
      :effect (and
            (at end (checked ?hazard)))
)

)