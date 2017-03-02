# robot_localization_2017
This is the base repo for the robot  localization assignment in CompRobo, spring 2017.
#### Kim Asenbeck, Brenna Manning, Jordan Van Duyne


### Project Goal

A robot in a known environment should be able to figure out where it is within that environment. This is the robot localization problem: given a map of an environment and sensor readings that provide information about the robot’s current location, a robot should be able localize itself (figure out where it is relative to the map). Our goal was to implement an algorithm, specifically the particle filter algorithm, to achieve localization of a Neato robot equipped with a lidar laser, which provides distance-to-object readings for 360˚ around the laser.

In addition to actually writing out a localization algorithm, another goal of this project was to gain more familiarity with ROS and robotics programming in general. 


### Solution to the problem

As previously mentioned, we implemented the particle filter algorithm to localize a Neato in a known map. The particle filter works through the following steps:

Input an initial guess of the robot’s location. This initial position guess consists of the x and y coordinates and heading of the robot relative to the map.
Create a “cloud” of particles around the initial position with a Gaussian distribution. Each particle  represents a possible location (x, y, and heading) that the robot could be in at that moment. The particles are distributed around the initial guess, instead of exactly at the initial guess, to introduce noise around the original guess: the initial guess is just a *guess*, so distributing particles around this guess compensates for the imperfection of the guess. We used 300 particles by default.
The robot moves (there is a change in the robot’s x, y, or heading), and the amount by which it moves is known.
The position of every particle is updated by this same amount that the robot moved. TODO: TALK ABOUT ADDING NOISE HERE (don’t wanna actually write this yet since we haven’t actually done it…)
A laser scan is taken and the distance readings are read. 
The likelihood of every particle’s location being correct, called the particle’s “weight”, is calculated using this laser scan data. The laser scan provides us with information - that is somewhat imperfect due to inaccuracies of the sensor - about how close the robot actually is to an object for every direction around the robot. Since each particle has an exact position and since we know the map, we can determine what the distance is to an object for every particle (and in every direction around a particle). This answers the question, “If the robot were actually located at this particle, what would the laser scan distance readings be?” If the difference between the robot’s actual distances and a particle’s distances is small, then the particle has a high probability of being located very close to the robot’s true location! If the difference is large, this particle is probably not in the right location. The particles’ weights are set to be this difference taken from a Gaussian distribution to introduce noise (accounting for inaccuracies in the laser sensor).
The particles’ weights are normalized so they sum to 1
300 new particles are created to replace the old particle cloud. The location of each new particle is determined by randomly selecting the location of one of the old particles. The probability that an old particle’s location will be chosen as a new particle’s location is determined by the old particle’s weight. Thus, there is a high chance that many new particles will spawn at the location of an old particle that was highly likely to be the robot’s actual position, whereas very few or even no particles will spawn in the locations of old particles that were unlikely to be the robot’s position.

These steps, except for creating an initial guess, continue to cycle through. The particle filter’s determination of the Neato’s location is simply the average of all of the particles’ locations. Over time, all of the particles will converge to being located among only a few different, highly likely locations, causing the average and the approximation of the Neato’s location to be (hopefully) fairly accurate. The particles should not all converge to the same spot due to the noise built in to the algorithm; there is a chance that a particle’s location could be very wrong, so we do not want all particles to accidentally all converge to one very wrong location, as recovering from such an inaccuracy would be nearly impossible.


### Design Decisions

  An interesting design decision we made was how we originally implemented the update_particles_with_laser function. Initially to set the weight for each particle, we determined where the nearest obstacle was to that particle, and compared that distance to the minimum non-zero laser scan distance from the neato. Once we had determined that distance d, the particle’s weight was set to e^(-(d^2)/2*(sigma^2)) with a sigma of 0.5.  After all particle weights were set in this way, they were normalized, and the particles were resampled.  This method was effective. Our particle filter was able to accurately determine the location of the robot in space.Another benefit of this implementation was how fast it was computationally. The speed allowed up to initialize our filter with many more particles than we otherwise would have. 

![original implementation mid](https://github.com/jovanduy/robot_localization_2017/blob/master/my_localizer/images/original_mid.png?raw=true)

![original implementation end](https://github.com/jovanduy/robot_localization_2017/blob/master/my_localizer/images/original_end.png?raw=true)
 
 
![original distance comparison](https://github.com/jovanduy/robot_localization_2017/blob/master/my_localizer/images/RobotLocalizationOriginal%20(2).png?raw=true)


  We were happy with the results of this implementation, but we did notice that while the x and y positions of particles were very accurate, the headings, or angles in space of the particles were less accurate. This encouraged us to think about ways we might improve how we determine particle weights.

One idea we had, was simply to compare the heading of the particle relative to the closest perceived obstacle to the heading of the neato relative to the closest perceived obstacle. This angle would be the index of the minimum of the ranges received from the lidar scan data. We then could have the weight determined by both the difference in distances and the difference in angles.

As an alternative, we modified the sigma value of the gaussian curve used to determine the headings of the particles when they are initialized at the beginning and after an initial guess is made. Provided that the initial guess is made facing around the correct direction, the particles will have reasonably accurate headings throughout the localization process, as they are updated continually using the update_particles_with_odom function.  This implementation is included as a script in our repository under the name pf_original.py. We did not make the change outlined in the visual above to incorporate the angles  into our weight calculation, but we believe it would also have been effective at eliminating the variation in particle headings. We did not make this change because around this point in the project we had a conversation with our professor about our implementation ideas, and based on that conversation we decided to try a new idea instead. 

  In our new implementation of update_particles_with_laser, instead of calculating the distance between the particle and the the nearest obstacle to that particle, and comparing that to the distance between the robot and the obstacle it is closest to, we are comparing something else. We are calculating where the obstacle would be located in the map coordinate frame based on the lidar data from the neato if the robot was located at the position and heading of the particle. We find the difference between where the obstacles should be for each angle if the neato is at the particle’s location and where the actual nearest obstacles to those points are in the map frame.  The weight of each particle is set based on these differences. 

!['new implementation mid'](https://github.com/jovanduy/robot_localization_2017/blob/master/my_localizer/images/new_mid.png?raw=true)

!['new implementation end'](https://github.com/jovanduy/robot_localization_2017/blob/master/my_localizer/images/new_end.png?raw=true)

  This new implementation was also effective at finding the location of the robot in the map. It was interesting to us that our original implementation appeared to be more accurate than the later implementation, which took more information into account when assigning weights.  This was the case for the map we were testing on at least. We believe that depending on the shape of the map, one or the other could be the better way to determine location. For example, using a long but narrow symmetrical map area, our original implementation would be less good, because the nearest obstacle would always be to the side, and the information about the longer distance would not be taken into account at all, but for the map we were testing with, that implementation was very good and was notably faster.

### Challenges

  Our biggest challenge was getting started on the assignment. We found it challenging to understand how all of the puzzle pieces fit together, and to know where to get started. Taking time to create a checklist of TODOs was very helpful in helping us get started. We also found it helpful to decide on an order of implementation at the start. From that point on, we found that each of the functions was relatively simple to implement. 

### Future Work

  Given more time, we would improve our particle filter’s accuracy and computational complexity. As our algorithm currently stands, we compute weights for every particle by examining all 360 elements in msg.ranges. We could improve the speed and accuracy of the particle filter by trying a number of things. First, we could experiment with using only select elements of ranges. For example, this may mean checking every second or tenth range. This benefits us by reducing the number of computations that need to be done at each step, but this approach runs the risk of skipping important measurements. Another drawback of this approach is that by limiting data, we run a greater risk of getting bad laser scan data. 
	Another interesting task that we would have tackled given more time is implementing the third idea we had for the update_particles_with_laser_scan algorithm. This third algorithm would have been a combination of our first, more naive, implementation and the final implementation we ended up with. In other words, this third approach would have combined the simplicity of the first implementation with the added accuracy of considering headings in a simple way. 
	Two additional tasks that we would have taken on given more time include making a map that takes advantage of the upward facing camera to build a map pose. In addition, we would have been interested in playing around with the sliding bars in RqTGuI in order to experiment with optimizing parameter values. While playing around with parameters, we might also experiment with adding noise in different places. For example, one of the places where we decided not to add noise was in update_particles_with_odom. Given more time, we would have experimented to find the optimal amount of noise here and throughout the code. 
	Finally, it would have been very interesting to do some more rigorous comparisons between these three approaches. This may have yielded interesting lessons about how each version of update_particles_with_laser_scan algorithm compares against the others, and provide insight about the value and tradeoffs of the added complexity. 

### Lessons

  Early on in this project we learned how valuable it is to understand what you need to do. Breaking things up into incremental steps is super helpful. We had to be sure to read through and understand the problems before diving in.  It was helpful for us that all team members were invested in the success of our project. It was productive for us to have regular working meetings with all team members rather than splitting up work to do individually. This way everyone always understood what was going on and was accountable for their work.   Additionally, we learned that there is often more than one way to do things. There isn’t always one true correct answer, and trying out different methods can help you gain a better understanding of what ways of solving problems work best depending on your situation.


