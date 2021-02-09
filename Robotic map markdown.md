# Robotics
Topics covered in the course of Robotics @University of Pisa

Programme of A.Y. 2019/20


## 1. Fundamentals

### Definition of robot

- A robot is an autonomous system which exists in the physical world, can sense its environment, and can act on it to achieve some goals
- Components

	- Actuators
	- Sensors
	- Controller

### Basic concepts

- Joint

	- Set of two surfaces that can slide keeping contact
	- Types

		- Rotational (revolute)
		- Translational (prismatic)

- Link 0 

	- Support base (origin of the reference frame)

### DOF

- is any of the minimum number of coordinates required to completely specify the motion of a robot
- = couple joint-link
- A rigid body in 3D space has 6DOF

	- 3 translational (x, y , z)
	- 3 rotational (roll, pitch, yaw)

- Robot

	- Controllable 

	  One actuator for every DOF.
	  Controllable DOFs = Total DOFs
	  

		- Can control all its DOFs
		- = Holonomic 

	- Uncontrollable

	  Controllable DOFs < Total DOFs
	  

		- Has more DOFs than it can control
		- = Non-holonomic

### Workspace

- Region described, from the origin of the end effector, when all the joints execute all the possible movements.
- It depends on

	- Lenght of the links
	- ROM of the joints

- Types

	- Reachable

		- region of space that the end effector can reach with at leas one orientation

	- Dextrous

		- region of space that the end effector can reach with more than one orientation

### Manipulator

- Open kinematic chain
- Sequence of links connected by joints and actuated by a motor
- Types

	- Antropomorphic (or rotational) 

		- RRR

	- Spherical

		- RRT

	- SCARA

		- RRT

	- Cilindrical

		- RTT

	- Cartesian

		- TTT

## 2, 3. Mechanics and Kinematics

### Space

- Joint (or configuration)

	- q = [ q0, q1, ..., q5]

- Cartesian (or operational)

	- x = [x, y, z, roll, pitch, yaw]

### Kinematic

Analytical study of the geometry of the arm motion, with respect to a steady Cartesian reference frame, without considering forces and torques which generate motion (actuation, inertia, friction, gravity, etc.). 

Analytical description of the relations between joint positions and the robot end effector position and orientation. 


- Direct

	- From joint to cartesian
	- x = K(q)

- Inverse

	- From cartesian to joint
	- q = K^-1 (x)

### Redundancy

- When DOFs > 6
- Dimension of operational space < dimension of joint space

### Transformation matrices

- Rotation
- Translation

### Denavit-Hartenberg representation

- Definition
- Parameters

	- a
	- alpha
	- d
	- theta

- Algorithm
- Examples

### Differential Kinematics

## 5, 6, 9. Biped locomotion

### Challenges

### Locomotion

- Passive
- Active

	- Humanoid robot kinematics

		- Newton-Raphson method

	- Humanoid robot dynamic

		- ZMP

			- In 2D
			- In 3D

				- Vertical component
				- Horizontal component

			- Measurement

		- Biped walking

			- Static
			- Dynamic

				- Pattern generation + Stabilization mechanism

					- Linear inverted pendulum model

## 10, 13. Neurocontrollers

### Learn the Inverse Kinematic

- For simple robots

	- Model

		- Feed-forward NN (es. MLP)

	- How to collect data for the training set

		- Experimentally
		- Exploiting the direct kinematics (computed with N-R method)

- For complex robots (es. soft robots) and in case of redundancies

	- FFNN with delays (small variations)

		- Also useful for

			- Offline prediction
			- Online prediction

### Learn the dynamic

- Direct dynamic

	- Discretize the model

		- Recurrent NN

			- NARX

				- Training phase 1: open loop

					- FFNN

				- Training phase 2: close the loop

					- feedbacks create a recursive structure

- Inverse dynamic

## 7. Sensors

### Definition of sensors and transductor

### Properties

- Transfer Function
- Calibration
- Histeresis
- Linearity
- Accuracy
- Repeatability
- Resolution
- Sensitiveness
- Noise
- Stability

### Role

- Exteroception
- Proprioception

### Where to place a sensor and why

- Before motor+reducer
- After motor+reducer

### Types

- Position sensors

	- Switches
	- (Optical) Encoders

		- Incremental
		- Absolute

	- Potentiometers
	- Hall effect

- Distance sensors

	- Triangulation

		- Passive (cameras)
		- Active (camera+laser)

	- Stereo vision
	- Ultrasound sensors
	- Laser range finders

- Proximity sensors

	- Passive (Hall effect)
	- Active (infrared)

- Force sensors

	- Strain gauge (estensimetro)
	- Force/Torque sensors

- Inertia

	- Accelerometers (linear acc.)
	- Giroscope (angular velocity)

## 8. Control (of Manipulator)

### One joint motion

- PID

	- Proportional term
	- Derivative term
	- Integral term

### Whole manipulator (multiple joints) motion

- Trajectory planning (trapezoidal velocity profile)

	- Different output for joint or cartesian space

- Motion control

	- In joint space

		- PD control + gravity compensation + inverse dynamic

	- In cartesian space

		- Based on transpose Jacobian
		- Based on inverse Jacobian

- Force control

## 16. Navigation

### Challenges

- Where I am

	- Localization

		- Odometry
		- Active beacons
		- Map matching
		- Landmarks

- Where are the object (obstacles)

	- Maps and models

		- Geometric

			- Occupancy grid

				- Fixed
				- Variable

			- Geometric description

		- Topological
		- Topometric (hybrid)

- How to reach a final position

	- Path Planning

		- for geometrical maps

			- Roadmaps

				- Visibility graphs
				- Voronoi diagrams

			- Cell decomposition
			- Potential Fields

		- for topological maps

			- directives -> instructions -> commands

	- Path Following

		- problems

			- non-omnidirectional robots
			- unexpected obstacles

				- detect them

					- distance sensors

				- avoid them

					- occupancy grid
					- potential fields

## 17. Architectures for behavior

### Robotics families

- Techniques for

	- Control
	- Perception & action

- Environment

	- Structured 
	- Unstructured

- Used for

	- Factories
	- Service/social robots

- Behaviour

	- Low-level
	- High-level

- Close loop to

	- Proprioceptive sensors
	- Exteroceptive-sensors

### How to control high-level behaviour

- Paradigms

	- Hierarchical

		- Model based architecture

	- Reactive

		- Behavior based architecture

			- Situated agent
			- Emergent behavior
			- Locality
			- Independence

	- Hybrid
	- Predictive

- How to describe the paradigms

	- Primitive functions

		- Sense
		- Plan

			- Strategic
			- Tactical
			- Executive

		- Act

	- Information flow

- How to manage concurrency of behaviors

	- Subsumption architecture (priorities)

		- Inhibition
		- Suppression

- Paradigms

	- Relation between primitives

		- Sense - Plan - Act
		- Sense - Act

	- Information flow

		- sensory data - info - directives - commands
		- sensory data - info - commands

	- Architecture

		- Deliberative (model based)
		- Reactive (behaviour based)

	- Inventor

		- Brooks

	- Prior knowledge

		- Internal model of the world
		- None

	- Advantages

		- Predictability
		- Faster, more reactive (real-time) , modular (parallel)

	- DIsadvantages

		- Time scale, Space, Information, Use of plans
		- Unpredictability, concurrency of behaviors

	- "Plan" structure

		- Horizontal, sequential
		- Vertical, parallel

## 18. Bioinspired 

### Bioinspiration and biomimetics

- Relation between robotics and biology
- Robotics Approaches

	- Focus on

		- Brain and central processing
		- Interaction with environment (emergent cognition)

	- Sensors

		- Robot vision
		- Bioinspired vision

	- Movement

		- Robot mechanics and kinematics
		- Embodied intelligence, soft robots

	- Control

		- Robot control
		- Neurocontrollers

	- Behavior

		- Robot behavior
		- Predictive behavior

	- Navigation

		- Robot navigation
		- Bioinspired navigation

### Embodiment

- Agent design principles

	- 1. Three-constituent principle

		- Environment 
		- Task
		- Body 

	- 2. Complete agent principle

		- Properties of a complete agent

			- Subject to the laws of physics
			- Generate sensory stimulation
			- Affect the environment
			- Are complex dynamical systems

				- Tend to settle into attractor states

			- Perform morphological computation

				- Mechanical feedback
				- is given by

					- body structure and shape
					- mechanical properties
					- arrangement of sensors

				- we must distinguish

					- Morphology facilitating control

						- es. passive walking

					- Morphology facilitating perception

						- es. compound eye

					- Morphological computation (proper term)

						- reservoir computing

	- 3. Cheap design
	- 4. Redundancy
	- 5. Sensory-Motor Coordination (active perception)
	- 6. Ecological balance
	- 7. Parallel processes
	- 8. Value

### Soft robots control

- Robot distinction

	- Discrete (rigid)

		- Model based (kinematic and dynamic)

		  Possible also for soft robot but definitely too complex (too much approximated and so error possibility is high).
		  Moreover they are too computationally expensive.
		  

	- Continuous (soft)

		- Learning based (neurocontrollers)

			- Supervised
			- Unsupervised
			- Reinforcement
			- Reservoir computing

### Perception-Action Loops

- Dealing with biological delays
- Predictions and anticipations

	- Comparison between real and expected perception

## 19-23. Vision

### Biological

Vision is a creative process. It's a perception system influenced by emotions.

The that starts from low level features extraction (orientation, color, contrast, direction) and it is linked to intermediate level (surface, shape discrimination, depth, segment) and high level processing (object recognition) by intermediate connections of the low level features.


- Retina

	- Transduction 

	  Early preprocessing of informations
	  

		- Light becomes electrical potential

			- Spikes (neuron activation)

		- Light goes through levels

			- Levels

				- Ganglion cells

				  Output neurons of the retina: they create spikes
				  
				  

					- Circular receptive field

						- Antagonistic behavior

							- Center ON
							- Center OFF

				- Bipolar cells
				- Photoreceptors

			- Difference from camera system

		- Chemical process

- Lateral genicular nucleus

	- 6 layers

		- 4 for color contrast & spatial frequencies
		- 2 for brightness contrast & temporal frequencies 

- Visual cortex

	- Linear receptive field
	- Cells

	  Detect orientations
	  

		- Simple cells
		- Complex cells

- Pathways

	- Dorsal (parietal) pathway

		- Motion and depth

	- Ventral (temporal) pathway

		- Color, orientation, complex shapes

### Artificial

Transduction of light intensities (gradients)


- Digital image

	- Basic concepts

		- Resolution
		- Depth
		- Channels
		- Intensity

			- I (u,v) = [R, G, B]
			- I (u,v) = [H, S, V]

		- Histogram

			- Represents the grey level occurrencies of an image ( = pixel intensities)

				- peaks = more white

			- Lost info about position (which pixel had that intensity)

	- Operations

		- Pixel operators

			- Monadic operations

				- Changes the distribution of grey level form Input to Output image
				- O (u, v) = f ( I [u, v] )

					- Some transformation functions can be reversed
					- Ex. the function can be a sigmoid (used to increase contrast)

				- Histogram equalization operation

				  L’obiettivo dell’equalizzazione dell’istogramma è quello di ridistribuire i valori dei livelli in modo che l’istogramma dell’immagine modificata sia quanto più uniforme possibile. In particolare osservando che l’istogramma cumulativo di un’immagine con distribuzione uniforme è una retta lineare l’obiettivo si traduce nel trovare un operazione puntuale che shifti le linee dell’istogramma cosicchè il risultante istogramma cumulativo sia approssimativamente lineare. 
				  
				  -----
				  
				  L’equalizzazione consiste nell’alterare i valori di ciascun pixel in modo che la numerosità di ogni livello non si discosti troppo da quella degli altri livelli, cioè in pratica “appiattisce”, per quanto possibile, l’istogramma. L’effetto è un aumento del contrasto per immagini con alcune caratteristiche particolari (per esempio radiografie, o immagini sottoesposte o sovraesposte).
				  

					- f ( I [u, v] ) = c ( I [u, v] )

						- c = cumulative distribution for a certain grey level  

			- Diadic operations 

				- Takes two pixel in input (not one) and gives one in output
				- Ex. green screen effect
				- High dynamic range images (HDRI)

					- Enhance some areas of the image where you do not have full infos

						- each sensor has a specific dynamic range (i.e. can capture only a certain range of illuminance)
						- merge different versions of the same image, to have full informations

					- Link

				- Background subtraction

		- Spatial (or Local) operators

			- The output is a function not of a single pixel or two, but of a window w of size i x j 
			- Convolution

				- 1D
				- 2D

					- Pointwise multiplication between filter and image
					- Gaussian filter

						- Corresponds to the center of the receptive field in retina

							- It creates antagonist behavior between center and periphery

								- Gaussian - Centre ON

									- Smoothing

								- Laplacian of a Gaussian - Centre OFF

									- Edge detection

										- Link

				- Associative property

					- producing different filters

## Construction of perception is a complex process

### Low level processing

- orientation, color, contrast, direction

### Intermediate level processing

- surface, shape discrimination, depth, segment

### High level processing

- Object recognition

*XMind - Trial Version*
