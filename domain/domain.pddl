(define (domain blocks-domain)

(:requirements :strips :typing :fluents :disjunctive-preconditions)

(:types 
	block_t
)

(:predicates 
	(inhand ?block - block_t) 
	(emptyhand) 
	(not_emptyhand) 
	(on ?block ?on_block - block_t) 
	(clear ?block - block_t)
)

(:action pick_up
    :parameters (?block ?from_block - block_t)
    :precondition (and 
		(emptyhand) (clear ?block) (on ?block ?from_block))
    :effect (and 
		(inhand ?block) (clear ?from_block) (not (emptyhand)) (not_emptyhand) (not (on ?block ?from_block)))
)

(:action put_down
    :parameters (?block ?on_block - block_t)
    :precondition (and 
		(not_emptyhand) (clear ?on_block) (inhand ?block))
    :effect (and 
		(on ?block ?on_block) (emptyhand) (not (not_emptyhand)) (not (inhand ?block)) (not (clear ?on_block)))
  )
)

