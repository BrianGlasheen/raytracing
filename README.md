# C++ Raytracer / Software Rasterizer
![Alt Text](https://github.com/BrianGlasheen/raytracing/blob/main/resources/output_13.gif?raw=true)
Artifacts from .gif format

this code is really gross and kinda broken for objs files, i want to go back to refactor and expand on this at some point + some shading is wrong on obj files

for best results use a res of >= 256 pixels, 
its multithreaded so they dont take too long

to run
./A6 [scene num] [image size] [output filename]
output filename doesn't need '.png', it's added

## scene 1/2

3 spheres with one light
![Alt Text](resources/output_1.png)

## scene 3

ellipse with sphere + multiple light sources
![Alt Text](resources/output_3.png)

## scene 4

single depth reflection
![Alt Text](resources/output_4.png)

## scene 5

mutli depth reflection
![Alt Text](resources/output_5.png)

## scene 6

default bunny
![Alt Text](resources/output_6.png)

## scene 7

transformed bunny
![Alt Text](resources/output_7.png)

## scene 8

scenes 1/2 transformed!
![Alt Text](resources/output_8.png)

## scene 9

blinn phong reflective surfaces
![Alt Text](resources/output_9.png)

## scene 10

blinn phong rough (reflection blending) reflective surfaces
![Alt Text](resources/output_10.png)

## scene 11

refraction + prev
![Alt Text](resources/output_11.png)


## scene 12

more refraction + prev
![Alt Text](resources/output_12.png)


## scene 13

gif output of rolling glass ball
![Alt Text](https://github.com/BrianGlasheen/raytracing/blob/main/resources/output_13.gif?raw=true)
