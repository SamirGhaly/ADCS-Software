# ADCS-Software
Attitude Determination and Control Subsystem (ADCS) of EUS-1\
Design and Implementation of ADCS SW subsystem of Egyptian Universities Satellite (EUS-1) \
The Project is a part of the Egyptian Universities Satellite Program provided by Egyptian Space Agency (EgSA).\
It was done as The Graduation Project for Aerospace Department Students at Faculty of Engineering, Cairo University\
By: Samir Samy / Mohammed Karam / Eman Ahmed / Ahmed Moussa / Ommar Hisham / Abdalla Gamal / Ammar Magdy\
Undersupervision of : Dr. Mohammed Lotfy

The software was designed to be suitable for application on any Satellite at any Orbit as:\
1- The modelling was done in a general way without any assumptions or special cases.\
2- Different estimation algorithms introduced.\
3- Different Control algorithms for different modes of operation [ Detumbling - Reorientation - Standby - Imaging ].\
4- Calculating PD Controller Gains was done using the method introduced in this paper: https://www.researchgate.net/publication/243772429_Quaternion_feedback_regulator_for_spacecraft_eigenaxis_rotation\


Note: the matlab files included needs to be merged for the code to run properly, I categorized the files for easier searching./
The Software was converted to simulink for better scalability, editability and optimization.
