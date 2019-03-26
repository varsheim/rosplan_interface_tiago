(define (domain guide)

(:requirements  :strips :typing :fluents :disjunctive-preconditions :durative-actions)

(:types locatable - communicating locatable - interactable moving - locatable human - moving robot - moving robot - controlled location); human - locatable robot - locatable location)

(:predicates (at ?obj - locatable ?loc - location ) (close ?loc1 - locatable ?loc2 - locatable) (interacts ?robot - moving ?human - moving) (follows ?entity1 - moving ?entity2 - moving) (followed ?entity1))

(:action GO
      :parameters (?obj - controlled ?start - location ?destination - location ?following - moving)
      :precondition (and (at ?obj ?start) (not (at ?obj ?destination)))
      :effect (and (at ?obj ?destination) (not (at ?obj ?start)) (when (follows ?following ?obj) (at ?following ?destination)) )
)

(:action APPROACH
      :parameters (?entity1 - controlled ?entity2 - locatable ?place - location)
      :precondition (and (at ?entity1 ?place) (at ?entity2 ?place) (not (close ?entity1 ?entity2)))
      :effect (and (close ?entity1 ?entity2))
)

(:action START_INTERACTION
      :parameters (?entity1 - interactable ?entity2 - interactable )
      :precondition (and (close ?entity1 ?entity2) (not (interacts ?entity1 ?entity2)))
      :effect (and (interacts ?entity1 ?entity2))
)

(:action INIT_FOLLOW
      :parameters (?entity1 - communicating ?entity2 - communicating )
      :precondition (and (interacts ?entity1 ?entity2) (not (follows ?entity2 ?entity1)))
      :effect (and (follows ?entity2 ?entity1) (followed ?entity1))
)
(:action END_FOLLOW
      :parameters (?entity1 - communicating ?entity2 - communicating )
      :precondition (and (interacts ?entity1 ?entity2) (follows ?entity2 ?entity1) (followed ?entity1))
      :effect (and (not (follows ?entity2 ?entity1)) (not (followed ?entity1)))
)
(:action END_INTERACTION
      :parameters (?entity1 - interactable ?entity2 - interactable )
      :precondition (and (close ?entity1 ?entity2) (interacts ?entity1 ?entity2) (not (follows ?entity2 ?entity1)) (not (follows ?entity1 ?entity2)))
      :effect (not (interacts ?entity1 ?entity2))
)

)
