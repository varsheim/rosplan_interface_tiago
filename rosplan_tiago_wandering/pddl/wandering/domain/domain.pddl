(define (domain wandering)

(:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions)

(:types
    robot-greet
    distance
    robot human item - locatable
    greet-location human-location item-location robot-location - location
)

(:predicates
    (empty_robot)
    (not_empty_robot)
    (item_on_robot ?item - item)
    (gave ?item - item ?human - human)
    (greeted ?greet - robot-greet ?human - human)
    (at ?obj - locatable ?loc - location)
    (linked_to_location ?lcbl - locatable ?loc - location)
    (linked_to_greet_location ?lcbl - locatable ?greetloc - greet-location)
)

(:durative-action GO
      :parameters (?obj - robot
                   ?start - location
                   ?destination - location)
      :duration ( = ?duration 60)
      :condition (and
            (at start (at ?obj ?start)))
      :effect (and
            (at end (at ?obj ?destination))
            (at start (not (at ?obj ?start))))
)

(:durative-action GREET
      :parameters (?obj - robot
                   ?greet - robot-greet
                   ?human - human
                   ?greetloc - greet-location)
      :duration ( = ?duration 8)
      :condition (and
            (over all (at ?obj ?greetloc))
            (over all (linked_to_greet_location ?human ?greetloc))) ;greetloc needs to be somehow connected to the action so the GO action can get this param
      :effect (and
            (at end (greeted ?greet ?human)))
)

(:durative-action GET_ITEM
       :parameters (?obj - robot
                    ?itemloc - item-location
                    ?item - item)
       :duration ( = ?duration 15)
       :condition (and
             (at start (empty_robot))
             (over all (at ?obj ?itemloc))
             (over all (linked_to_location ?item ?itemloc)))
        :effect (and
             (at start (not_empty_robot))
             (at start (not (empty_robot)))
             (at end (item_on_robot ?item)))
)

(:durative-action GIVE_ITEM
       :parameters (?obj - robot
                    ?item - item
                    ?human - human
                    ?humanloc - human-location)
       :duration ( = ?duration 10)
       :condition (and
             (at start (item_on_robot ?item))
             (at start (not_empty_robot))
             (over all (at ?obj ?humanloc))
             (over all (linked_to_location ?human ?humanloc)))
        :effect (and
             (at start (not (not_empty_robot)))
             (at start (empty_robot))
             (at end (not (item_on_robot ?item)))
             (at end (gave ?item ?human)))
)

)