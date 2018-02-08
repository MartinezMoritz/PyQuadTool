PYQUADTOOL

This project presents the design and development of drone dynamics toolbox implemented
in Python programming language, called PyQuadTool . This toolbox allows
the 3D simulation of a drone that is lifted and propelled by four rotors (so-called
quadrotor), and enables the user to define complex trajectories, dynamic models and
closed-loop control for trajectory tracking.
PyQuadTool is a simulation software which allows modelling, flight control and navigation
of aerial robots. As a whole, the PyQuadTool project consist of four major
stages of development:

• A 3D Graphics User Interface showing the current position and robot speed
at all times.
• Plant Modelling including Aerodynamic forces and quadrotor dynamics using
Rigid Body Dynamics and 6-dimensional Spatial Notation.
• Trajectory Planner: Trapezoidal footspeed profiles have been used for straightline
trajectory generation and cubic spline frontiers for curve-line trajectory
generation.
• The flight controller: Attitude, vertical altitude and position control that
allow the robot to follow complex trajectories.

INSTRUCTIONS:

PyQuadTool can run on Linux, Windows, and OSX. It should, however, run on any
platform which supports the following packages:

Python 2.7 and 3+
PyQt 4.8+ or PySide
PyQtGraph
NumPy
python-opengl bindings are required for 3D graphics

1) Install Python:
	https://www.python.org/

2) Install NumPy and SciPy libraries:
	http://www.scipy.org/

3) Install PyQt 4.8 or 5+:
	http://pyqt.sourceforge.net/Docs/PyQt5/installation.html

4) Install PyOpenGl:
	http://pyopengl.sourceforge.net/

5) Install PyQtGraph:
	http://www.pyqtgraph.org/

6) Run Index.py
