(define (domain blocks-domain)

(:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions)

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

(:durative-action pick_up
    :parameters (?block ?from_block - block_t)
	:duration ( = ?duration 15)
    :condition (and 
		(at start (emptyhand)) (over all (clear ?block)) (at start (on ?block ?from_block)))
    :effect (and 
		(at end (inhand ?block)) (at end (clear ?from_block)) (at start (not (emptyhand))) (at start (not_emptyhand)) (at start (not (on ?block ?from_block))))
)

(:durative-action put_down
    :parameters (?block ?on_block - block_t)
	:duration ( = ?duration 17)
    :condition (and 
		(at start (not_emptyhand)) (over all (clear ?on_block)) (at start (inhand ?block)))
    :effect (and 
		(at end (on ?block ?on_block)) (at end (emptyhand)) (at end (not (not_emptyhand))) (at start (not (inhand ?block))) (at start (not (clear ?on_block))))
  )
)

