# Schwarzschild_Raytracer
By Cecilia
This project implements a realtime raytracer in the Schwarzschild-metric (a sphere with a black hole inside).

It is written with Visual Studio in C++, but has no real dependence on it.  
It used FreeGlut 3.0.0 and GLEW 1.13.0 for OpenGl and SOIL (licenses included).

You can refer to my included bachelor thesis about background info:  

Feel free to modify, improve, or use functionality of this code (like proposed in chapter 7 of the theses, easy integration into conventional ray tracers)

Building is handled by cMake and vcpkg, which FirePrincess helped with.
Otherwise make sure you have the right VS-redistributable installed, all the dlls and the fragment shader in the same directory as the exe. 

https://www.eso.org/public/images/eso0932a/ is included in the project as default picture , credited to ESO/S. Brunier.
You can also manually place a picture in the executeable folder, whose name you enter into the console when executing the program. For pictures I recommend the NASA website.

###Controls:
Click the screen and move your mouse for view  

1 stationary mode  
2 free falling snapshot  
3 free fall  
4 orbit  

WASD horizontal movement  
QE   vertical movement  
N    switch to neutron star mode

P reset  
