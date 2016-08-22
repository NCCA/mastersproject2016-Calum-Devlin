This Project will run a single instance of a flock of boids.

All relevant parameters can be altered in the Scene.h file.
E.G.:	Quantity of boids;
	Speed of boids;
	Area of awareness per boid;
	Type of flock structure.

Presently, there are only 2 working structures:
A Naive structure, which compares every boid against every other boid (this is known to be relatively inefficient)
A Quadtree structure, which executes faster, but could still be improved.

A Delaunay Graph structure is in progress, and uses code from the naive method as a placeholder.


CONTROLS:
Optimal controls include a keyboard and 3-button mouse.
Use LMB to rotate the scene.
Press MMB to zoom in and out.
Use RMB to pan N/S/W/E.
Scroll MMB to change the zoom and pan factor.

Press W/S to switch between Wireframe and Solid rendering.
Press Spacebar to pause the simulation.
Press Esc to quit.
