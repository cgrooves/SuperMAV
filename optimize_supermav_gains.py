########################################################################
#
#     This is an example call of MIDACO 6.0
#     -------------------------------------
#
#     MIDACO solves Multi-Objective Mixed-Integer Non-Linear Problems:
#
#
#      Minimize     F_1(X),... F_O(X)  where X(1,...N-NI)   is CONTINUOUS
#                                      and   X(N-NI+1,...N) is DISCRETE
#
#      subject to   G_j(X)  =  0   (j=1,...ME)      equality constraints
#                   G_j(X) >=  0   (j=ME+1,...M)  inequality constraints
#
#      and bounds   XL <= X <= XU
#
#
#     The problem statement of this example is given below. You can use
#     this example as template to run your own problem. To do so: Replace
#     the objective functions 'F' (and in case the constraints 'G') given
#     here with your own problem and follow the below instruction steps.
#
########################################################################
######################   OPTIMIZATION PROBLEM   ########################
########################################################################
import matlab.engine

engine = matlab.engine.start_matlab()

def problem_function(x):

    f = [0.0]*4 # Initialize array for objectives F(X)
    g = [0.0]*4 # Initialize array for constraints G(X)

    # Enumerate inputs
    kp_chi = x[0]
    ki_chi = x[1]

    kp_phi = x[2]
    kd_phi = x[3]
    ki_phi = x[4]

    kp_h = x[5]
    ki_h = x[6]

    kp_theta = x[7]
    kd_theta = x[8]
    ki_theta = x[9]

    chi_gains = matlab.double([kp_chi,ki_chi])
    phi_gains = matlab.double([kp_phi,kd_phi,ki_phi])
    h_gains = matlab.double([kp_h,ki_h])
    theta_gains = matlab.double([kp_theta,kd_theta,ki_theta])
    V_gains = matlab.double([.234,.52])

    # Objective functions F(X)
    # Minimize rise time, settling time, overshoot for chi and h
    # Run matlab function, get values
    fun_out = engine.mav_response(chi_gains,phi_gains,h_gains,theta_gains,V_gains,0)

    out = fun_out[0]

    tr_chi = out[0]
    ts_chi = out[1]
    ov_chi = out[2]

    tr_h = out[3]
    ts_h = out[4]
    ov_h = out[5]

    ts_p = out[6]
    ts_q = out[7]

    # Objectives stated here
    f[0] = tr_chi
    f[1] = ov_chi

    f[2] = tr_h
    f[3] = ov_h

    #  Equality constraints G(X) = 0 MUST COME FIRST in g[0:me-1]
    # Inequality constraints G(X) >= 0 MUST COME SECOND in g[me:m-1]
    g[0] = ts_chi - 5
    g[1] = ts_h - 5
    g[2] = 50 - ts_chi
    g[3] = 50 - ts_h

    return f,g

########################################################################
#########################   MAIN PROGRAM   #############################
########################################################################

key = b'MIDACO_LIMITED_VERSION___[CREATIVE_COMMONS_BY-NC-ND_LICENSE]'

problem = {} # Initialize dictionary containing problem specifications
option  = {} # Initialize dictionary containing MIDACO options

problem['@'] = problem_function # Handle for problem function name

########################################################################
### Step 1: Problem definition     #####################################
########################################################################

# STEP 1.A: Problem dimensions
##############################
problem['o']  = 4  # Number of objectives
problem['n']  = 10  # Number of variables (in total)
problem['ni'] = 0  # Number of integer variables (0 <= ni <= n)
problem['m']  = 4  # Number of constraints (in total)
problem['me'] = 0  # Number of equality constraints (0 <= me <= m)

# STEP 1.B: Lower and upper bounds 'xl' & 'xu'
##############################################
problem['xl'] = [ 0, 0, 0, 0, 0, 0, 0, -5, -5, -5]
problem['xu'] = [ 5, 5, 5, 5, 5, 5, 5, 0, 0, 5]

# STEP 1.C: Starting point 'x'
##############################
problem['x'] = [0.88,.041,.88,.16,.304,.0181,.0039,-1.6,-.4338,0] # Here for example: starting point = lower bounds
# problem['x'] = problem['xu']

########################################################################
### Step 2: Choose stopping criteria and printing options    ###########
########################################################################

# STEP 2.A: Stopping criteria
#############################
option['maxeval'] = 10000     # Maximum number of function evaluation (e.g. 1000000)
option['maxtime'] = 60*30  # Maximum time limit in Seconds (e.g. 1 Day = 60*60*24)

# STEP 2.B: Printing options
############################
option['printeval'] = 1000  # Print-Frequency for current best solution (e.g. 1000)
option['save2file'] = 1     # Save SCREEN and SOLUTION to TXT-files [0=NO/1=YES]

########################################################################
### Step 3: Choose MIDACO parameters (FOR ADVANCED USERS)    ###########
########################################################################

option['param1']  = 0.0  # ACCURACY
option['param2']  = 0.0  # SEED
option['param3']  = 0.0  # FSTOP
option['param4']  = 0.0  # ALGOSTOP
option['param5']  = 0.0  # EVALSTOP
option['param6']  = 0.0  # FOCUS
option['param7']  = 0.0  # ANTS
option['param8']  = 0.0  # KERNEL
option['param9']  = 0.0  # ORACLE
option['param10'] = 0.0  # PARETOMAX
option['param11'] = 0.0  # EPSILON
option['param12'] = 0.0  # BALANCE
option['param13'] = 0.0  # CHARACTER

########################################################################
### Step 4: Choose Parallelization Factor   ############################
########################################################################

option['parallel'] = 0 # Serial: 0 or 1, Parallel: 2,3,4,5,6,7,8...

########################################################################
############################ Run MIDACO ################################
########################################################################

import midaco

if __name__ == '__main__':

  solution = midaco.run( problem, option, key )

  print(solution['f'])
  print(solution['g'])
  print(solution['x'])

########################################################################
############################ END OF FILE ###############################
########################################################################
