# vizbiz
Miniproject for building a visual odometry pipeline.

# FAQ
[Google Docs](https://docs.google.com/document/d/1RiG-70-2xwgPcNLGuMzIm5BZ4n8TPx_bFLPe546079E/edit "Whoever finds this last pays a round of coffe. ;) Sign here: Miro, Pascal, Fabio")

# How to run:
In order to run the visual odometry two launch procedures where implemented:

* Debug Mode: For development and debug mode the main.m with default parameters can be executed. Numerous individual plots are displayed with insightful information about matching, inlier rejection and triangulation.

* Simple GUI: Out of performance reasons a more compact and user-friendly display of the pipeline output was created with a GUI designed with the Matlab GUIDE application. Only the most crucial entities, like number of landmarks, are visualized for intuitive understanding.

# How to run the GUI
Please follow the steps below to run the visual odometry through the GUI environment:

* adapt dataset paths in loadParameters.m
* type into MATLAB command window: gui_simple
* in *Parameters* panel select dataset to run on and toggle respective radio buttons
* hit *Run* to trigger the visual odometry

Vision Algorithms for Mobile Robotics, 2016 ETHZ.
