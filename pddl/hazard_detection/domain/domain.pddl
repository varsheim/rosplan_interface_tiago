(define (domain hazarddetection)

(:requirements :strips :typing :disjunctive-preconditions :negative-preconditions)

(:types
    hazard
    robot - locatable
    hazard-location - location)

(:predicates
    (obj_at ?obj - locatable ?loc - location )
    (checked ?hazard - hazard)
    (linked ?hazard - hazard ?loc - hazard-location)
)

(:action GO
      :parameters (?obj - robot ?start - location ?destination - location)
      :precondition (and
            (obj_at ?obj ?start)
            (not (obj_at ?obj ?destination)))
      :effect (and
            (obj_at ?obj ?destination)
            (not (obj_at ?obj ?start)))
)
(:action CHECK
      :parameters (?obj - robot ?hazard - hazard ?hazloc - hazard-location)
      :precondition (and
            (obj_at ?obj ?hazloc)
            (linked ?hazard ?hazloc)
      		(not (checked ?hazard)))
      :effect (and
            (checked ?hazard))
)
)
