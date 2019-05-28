(define (domain wandering)

(:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions)

(:types
    robot-greet - linkable
    distance
    robot item human - locatable
    greet-location human-location item-location robot-location - location
)

(:predicates
    (empty_robot)
    (not_empty_robot)
    (item_on_robot ?item - item)
    (gave ?item - item ?human - human)
    (greeted ?human - human)
    (obj_at ?obj - locatable ?loc - location)
    (linked_to_locatable ?ln - linkable ?lcbl - locatable)
)

(:durative-action GO
      :parameters (?obj - robot
                   ?start - location
                   ?destination - location)
      :duration ( = ?duration 60)
      :condition (and
            (at start (obj_at ?obj ?start)))
      :effect (and
            (at end (obj_at ?obj ?destination))
            (at start (not (obj_at ?obj ?start))))
)

(:durative-action GREET
      :parameters (?obj - robot
                   ?greet - robot-greet
                   ?human - human
                   ?humanloc - human-location
                   ?from - distance)
      :duration ( = ?duration 8)
      :condition (and
            (over all (obj_at ?obj ?humanloc))
            (over all (linked_to_locatable ?greet ?human)))
      :effect (and
            (at end (greeted ?human)))
)

(:durative-action GET_ITEM
       :parameters (?obj - robot
                    ?itemloc - item-location
                    ?item - item)
       :duration ( = ?duration 15)
       :condition (and
             (at start (empty_robot))
             (over all (obj_at ?obj ?itemloc)))
        :effect (and
             (at start (not_empty_robot))
             (at start (not (empty_robot)))
             (at end (item_on_robot ?item)))
)

(:durative-action GIVE_ITEM
       :parameters (?obj - robot
                    ?item - item
                    ?human - human
                    ?humanloc - human-location
                    ?from - distance)
       :duration ( = ?duration 10)
       :condition (and
             (at start (item_on_robot ?item))
             (at start (not_empty_robot))
             (over all (obj_at ?obj ?humanloc)))
        :effect (and
             (at start (not (not_empty_robot)))
             (at start (empty_robot))
             (at end (not (item_on_robot ?item)))
             (at end (gave ?item ?human)))
)

)