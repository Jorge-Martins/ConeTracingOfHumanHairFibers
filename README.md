# Cone Tracing of Human Hair Fibers

Solution developed in the context of my Master thesis with the same name, focusing  on improving the rendering performance with two contributions: a cone based solution and a hybrid bounding volume hierarchy solution. With the cone based solution we are able to produce aliasing free images with just a super-sampling of 2x2 and produce images of comparable quality to the ones produced with a stochastic ray tracer with a 16x16 super-sampling rate while requiring a much lower rate and achieving speedups of up to 4. With the hybrid bounding volume hierarchy, which uses both axis aligned and oriented bounding boxes to bound hair primitives, we are able to achieve an average intersection test reduction of 53%, while only increasing the memory footprint by 11%. 

This solution provides you with:
- A stochastic ray tracer
- A cone tracer
- A LBVH with axis aligned bounding boxes
- A hybrid LBVH with both axis aligned and object oriented bounding boxes
- A console image comparator that provides the visual difference between two images as well as the RMS error value.

# Terms of Use
This code is made available for academic use only and under the requirement of providing proper acknowledgments of its consultation, use or extension. Please reference this repository "https://github.com/Jorge-Martins/ConeTracingOfHumanHairFibers" as well as the published work available at https://fenix.tecnico.ulisboa.pt/cursos/meic-a/dissertacao/283828618789642 and https://www.igi-global.com/article/efficient-hair-rendering-with-a-gpu-cone-tracing-approach/      
