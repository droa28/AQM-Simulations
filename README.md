# AQM Simulations 
Numerical simulations of Active Queue Management (AQM) schemes using Matlab-Simulink and NS-3.
- This work addresses the misconceptions regarding usage of the control-theoretic AQM feedback structure. This work has been submitted to a journal, see reference below.
- Our simulation files can be used to verify and validate the numerical time-domain and event-domain simulations of existing AQM schemes using the MATLAB-Simulink platform 
and the NS-3 platform respectively. 
- The simulation results are easily reproducible, and new AQM control algorithms can easily be compared to existing AQM schemes by using the simulation files in this repository.
# Matlab-Simulink Simulation Files
PID_JNLM_x1 : compares between Jacobi Model and Misra Model.\
PID_NLM_x2a : compares between different controllers using Misra Model.\
PID_NLM_x2b : same as PID_NLM_x2a except for the Kahe Controller. \
PID_NLM_x2c : same as PID_NLM_x2a except that the PID controller blocks uses du/dt block.\
AQM_param   : script file for network parameters and P(I)D controller parameters to be used by Simulink models.
# NS-3 Simulation Files (NS-3.30.1 Release)
wscript            : to be placed in "\\src\\traffic-control" folder.\
pid-script.cc      : simulation script file, to be placed in "\\scratch" folder.\
pid-queue-disc.h   : header file for queue disc, to be placed in "\\src\\traffic-control\\model" folder.\
pid-queue-disc.cc  : class file for queue disc, to be placed in "\\src\\traffic-control\\model" folder.\
tcp-tx-buffer.cc   : This file includes the patch for "assert error when sack is disabled", to be placed in "\\src\\internet\\model" folder.
# If part of this work is used, please cite as below
R. Olusegun Alli-Oke, On the validity of numerical simulations for control-theoretic AQM schemes in computer networks, *Mathematics and Computers in Simulation*, submitted 2020. GitHub repository: https://github.com/droa28/AQM-Simulations (Nov. 2020).
# Contact Information
I can be reached at razkgb2008@yahoo.com, rallioke@mit.edu, razak.alli-oke@elizadeuniversity.edu.ng

