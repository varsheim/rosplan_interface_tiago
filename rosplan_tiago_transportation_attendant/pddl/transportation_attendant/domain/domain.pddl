(define (domain transportationattendant)

(:requirements :typing :adl :durative-actions :numeric-fluents)

(:types robot human - attendable
        attendable item - locatable
        human-location item-location robot-location waypoint - location)

(:predicates (empty_robot)
             (not_empty_robot)
             (item_on_robot ?item - item)
             (gave ?item - item ?human - human)
             (at ?obj - locatable ?loc - location)
             (attended ?obj - attendable ?from - location ?to - location)
             (linked_to_location ?lcbl - locatable ?loc - location)
             (load_left ?item - item ?human - human ?loc - location))

(:durative-action GO
    :parameters (?obj - robot ?start - location ?destination - location)
    :duration (= ?duration 60)
    :condition (and
               (at start (at ?obj ?start))
               (over all (empty_robot)))
    :effect (and
            (at end (at ?obj ?destination))
            (at start (not (at ?obj ?start))))
)

(:durative-action GO_WITH_ATTENDANCE
    :parameters (?obj - robot ?human - human ?start - location ?destination - location)
    :duration (= ?duration 200)
    :condition (and
               (at start (at ?obj ?start))
               (at start (at ?human ?start)))
    :effect (and
            (at end (at ?obj ?destination))
            (at end (at ?human ?destination))
            (at end (attended ?human ?start ?destination))
            (at start (not (at ?obj ?start)))
            (at start (not (at ?human ?start))))
)

(:durative-action GET_LOAD
       :parameters (?obj - robot ?item - item ?human - human ?itemloc - item-location)
       :duration ( = ?duration 30)
       :condition (and
                  (at start (empty_robot))
                  (over all (at ?obj ?itemloc))
                  (over all (at ?human ?itemloc))
                  (over all (linked_to_location ?item ?itemloc)))
       :effect (and
               (at start (not_empty_robot))
               (at start (not (empty_robot)))
               (at end (item_on_robot ?item)))
)

(:durative-action LEAVE_LOAD
       :parameters (?obj - robot ?item - item ?human - human ?destination - location)
       :duration ( = ?duration 30)
       :condition (and
                  (at start (item_on_robot ?item))
                  (at start (not_empty_robot))
                  (over all (at ?obj ?destination))
                  (over all (at ?human ?destination)))
       :effect (and
               (at end (not (not_empty_robot)))
               (at end (empty_robot))
               (at end (not (item_on_robot ?item)))
               (at end (load_left ?item ?human ?destination)))
)

)