# WormRobotCodes
Codes used to run CMMWorm prototype

I will add more codes here as we publish significant results and improvements.

The codes above uses external libraries for using Dynamixel actuators with either an Arduino or OpenCM9.04C

->https://github.com/vanadiumlabs/arbotix/tree/master/libraries/Bioloid
->https://github.com/horchler/DynamixelQ
->http://mayhewlabs.com/media/Mux_Shield_II_User_Guide.pdf

1.CMMWorm_Probability_Slack_Detection_3_24 -> Works with OpenCM9.04C controller and dynamixel MX64 actuators
  To detect load anomalies on worm like robot using a probabilitic appraoch
  
  Kandhari, A., Horchler, A.D., Zucker, G.S., Daltorio, K.A., Chiel, H.J. and Quinn, R.D., 2016, July. Sensing contact constraints in a     worm-like robot by detecting load anomalies. 
  In Conference on Biomimetic and Biohybrid Systems (pp. 97-106). Springer, Cham.
  
2.CMMWorm_SS_Pipe_ver3.ino -> Works with an Arbotix board and dynamixel AX18A actuators
  To detect external constraints and segment state using sensors
  
  Kandhari, A., Stover, M.C., Jayachandran, P.R., Rollins, A., Chiel, H.J., Quinn, R.D. and Daltorio, K.A., 2018, July. Distributed         Sensing for Soft Worm Robot Reduces Slip for Locomotion in Confined Environments. 
  In Conference on Biomimetic and Biohybrid Systems (pp. 236-248). Springer, Cham.
  
3.SEC_ver2 -> Works with a Robotis OpenCM9.04 controller and dynamixel AX18A actuators
  To turn using non perioidic waves using slip elimination control method please refer to        https://github.com/kandhariakhil/NonPeriodicWaves
  for simulation.
  
  Kandhari, A., Wang, Y., Chiel, H.J. and Daltorio, K.A., 2019. Turning in Worm-Like Robots: The Geometry of Slip Elimination Suggests    Nonperiodic Waves. Soft robotics.
