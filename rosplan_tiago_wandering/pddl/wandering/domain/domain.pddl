(define (domain wandering)

(:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions)

(:types
    item
    hazard
    distance
    robot - locatable
    hazard-location - location
    greet-location - location
    human-location - location
    item-location - location
)

(:predicates
    (empty_robot)
    (not_empty_robot)
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

(:durative-action GREET
      :parameters (?obj - robot ?greetloc - greet-location ?human - human ?from - distance)
      :duration ( = ?duration 10)
      :condition (and
            (over all (obj_at ?obj ?greetloc)))
      :effect (and
            (at end (greeted ?human)))
)

(:durative-action GET_ITEM
       :parameters (?obj - robot ?itemloc - item-location ?item - item)
       :duration ( = ?duration 10)
       :condition (and
             (at start (empty_robot))
             (over all (obj_at ?obj ?itemloc)))
        :effect (and
             (at end (greeted ?human)))
)

(:durative-action GIVE_ITEM
       :parameters (?obj - robot ?greetloc - greet-location ?side - human-location)

)


)
