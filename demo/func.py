import sys, time
from pyscipopt import Model
from Parameter import *
import random
import matplotlib.pyplot as plt
import matplotlib.patches as patches


def printSolution(optimal_state_sequence, trigger_sequence):
    x = range(0, 21) 
    y = [i[0] for i in optimal_state_sequence]  

    plt.plot(x, y)

    # Subplot 2
    plt.plot(x, y, label='state trajectory', color='orange')
    for i in range(len(trigger_sequence)):
        s = trigger_sequence[i]
        if i == 0:
            plt.scatter(s, y[s], c='black', label = "starting point")
        else:plt.scatter(s, y[s], c='black')
    rect4 = patches.Rectangle((0,12),8,2,linewidth=1,edgecolor='g',facecolor='none', hatch='//')
    rect5 = patches.Rectangle((0,5),8,10,linewidth=1,edgecolor='b',facecolor='b', alpha=0.5)
    rect6 = patches.Rectangle((12,13),8,10,linewidth=1,edgecolor='r',facecolor='r', alpha=0.5)
    rect7 = patches.Rectangle((12,11),8,4,linewidth=1,edgecolor='brown',facecolor='none', hatch='//')
    plt.gca().add_patch(rect4)
    plt.gca().add_patch(rect5)
    plt.gca().add_patch(rect6)
    plt.gca().add_patch(rect7)
    plt.text(4, 13, 'A1', ha='center', va='center')
    plt.text(4, 10, 'A2', ha='center', va='center')
    plt.text(16, 18, 'A3_1', ha='center', va='center')
    plt.text(16, 13, 'A3_2', ha='center', va='center')
    plt.xlabel('time')
    plt.title('Altitude')
    plt.legend()
    plt.show()

def addConsmymax(model, myvar, mymaxnum):
    z_ind={}
    num = len(myvar)
    for i in range(num):
        z_ind[i] = model.addVar(vtype="B", name="indicator(%s)"%i)
        model.addCons(myvar[i]<=mymaxnum, 'gthanall')
        model.addCons(myvar[i] + 100000*(1-z_ind[i]) >= mymaxnum ,'equone')
    model.addCons(sum(z_ind[i] for i in range(num)) == 1, 'sum_equ1')

def addConsmymin(model, myvar, myminnum):
    z_ind={}
    num = len(myvar)
    for i in range(num):
        z_ind[i] = model.addVar(vtype="B", name="indicator(%s)"%i)
        model.addCons(myvar[i] >= myminnum, 'lessthanall')
        model.addCons(myvar[i] - M*(1-z_ind[i]) <= myminnum ,'equone')
    model.addCons(sum(z_ind[i] for i in range(num)) == 1, 'sum_equ1')

def predict(curr_t, curr_s, horizon):
    print("predicting at instant:{}".format(curr_t))
    # print("disturb_set: {}".format(disturb_set))
    model = Model("predictor")
    # model.setPresolve(SCIP_PARAMSETTING.OFF)
    model.setParam("numerics/feastol", 1e-4)
    model.setRealParam("limits/time", 60)
    model.hideOutput()
    z, u = {}, {}
    # add variants

    for i in range(horizon + 1):
        z[i, 0] = model.addVar(lb = 0, ub = 100, vtype="C", name="z")  # system state
        z[i, 1] = model.addVar(lb = -5, ub = 5, vtype="C", name="z")  # system state
    for i in range(horizon):
        u[i, 0] = model.addVar(lb=-2.5, ub=2.5, vtype="C", name="u0")  # control input

    t = {}
    Add_model_constrs(model, z, u, curr_s, horizon)
    Add_robust_constrs(model, z, curr_t, horizon, t)
    Add_obj(model,u,horizon)
    model.optimize()
    status = model.getStatus()
    if status == "optimal":
        # print("optimal with disturb:{} ".format(disturb_set))
        state, input = {}, {}
        for i in range(horizon + 1):
            state[i, 0] = model.getVal(z[i, 0])
            state[i, 1] = model.getVal(z[i, 1])
        for i in range(horizon):
            input[i, 0] = model.getVal(u[i, 0])
        # print("rho and obj are: {}".format([model.getVal(p) for p in t.values()])) 
        print("predicted at curr_t:{} is {}".format(curr_t, [state[i, 0] for i in range(horizon+1)]))
        # print("input at curr_t:{} is {}".format(curr_t, input))
        return [input[i, 0] for i in range(horizon)]
    else:
        print('Optimization was stopped with status ' + str(status))
        sys.exit(0)
        
        

            
def Add_model_constrs(model, z, u, curr_s, horizon):
    model.addCons(z[0, 0] == curr_s[0])
    model.addCons(z[0, 1] == curr_s[1])
    model.addConss((z[i + 1 , 0] == z[i , 0] + 0.5*z[i, 1]+0.5*u[i, 0] for i in range(horizon)))
    model.addConss((z[i + 1 , 1] == z[i , 1] + u[i, 0] for i in range(horizon)))
def Add_robust_constrs(model, z, curr_t, horizon, t):
    rho_physical_1, rho_physical_2 = {}, {}
    rho_F0, rho_G0, rho_U0 = {}, {}, {}
    rho_F0_1, rho_G0_1, rho_U0_1, rho_U0_2 = {}, {}, {}, {}
    F_1, G_1, U_01, U_02, U_03, U_0 = {}, {}, {}, {}, {}, {}
    for i in range(horizon+1):
        for j in range(2):
            rho_physical_1[i,j]=model.addVar(lb = None, ub = None, vtype = "C", name = "rho_physical_1")
        rho_physical_2[i] = model.addVar(lb = None, ub = None, vtype = "C",name = "rho_physical_2")
    rho_physical = model.addVar(lb = None, ub = None, vtype = "C",name = "rho_physical")
    for i in range(horizon+1):
        model.addCons(rho_physical_1[i, 0] == z[i, 0] - 0)
        model.addCons(rho_physical_1[i, 1] == 100 - z[i, 0])
        addConsmymin(model, [rho_physical_1[i, j] for j in range(2)], rho_physical_2[i])
    addConsmymin(model, [rho_physical_2[i] for i in range(horizon+1)], rho_physical)
    #------F0[0, 8]A1
    if(curr_t <= F0_max and 1 in I):
        F0_horizon = min(F0_max-curr_t, horizon)
        rho_F0 = model.addVar(lb = None, ub = None, vtype="C", name="rho_F0")
        for i in range(F0_horizon + 1):
            for p in range(2):
                rho_F0_1[i, p] = model.addVar(lb = None, ub = None, vtype="C", name="rho_G0_1")
            F_1[i] = model.addVar(lb = None, ub = None, vtype="C", name="G_1")
        for i in range(F0_horizon+1):
            model.addCons(rho_F0_1[i, 0] == z[i, 0] - A1[0])
            model.addCons(rho_F0_1[i, 1] == A1[1] - z[i, 0])
            addConsmymin(model, [rho_F0_1[i, 0], rho_F0_1[i, 1]], F_1[i])    
        addConsmymax(model, [F_1[i] for i in range(F0_horizon + 1)], rho_F0)
    #------G0[0, 8]A2
    if(curr_t <= G0_max and 2 in I):
        G0_horizon = min(G0_max-curr_t, horizon)
        rho_G0 = model.addVar(lb = None, ub = None, vtype="C", name="rho_G0")
        for i in range(G0_horizon + 1):
            for p in range(2):
                rho_G0_1[i, p] = model.addVar(lb = None, ub = None, vtype="C", name="rho_G0_1")
            G_1[i] = model.addVar(lb = None, ub = None, vtype="C", name="G_1")
        for i in range(G0_horizon+1):
            model.addCons(rho_G0_1[i, 0] == z[i, 0] - A2[0])
            model.addCons(rho_G0_1[i, 1] == A2[1] - z[i, 0])
            addConsmymin(model, [rho_G0_1[i, 0], rho_G0_1[i, 1]], G_1[i])    
        addConsmymin(model, [G_1[i] for i in range(G0_horizon + 1)], rho_G0)
    #------A3_1U0[12,20]A3_2
    if(curr_t+horizon >= U0_min and curr_t <= U0_max and 3 in I):
        if (curr_t>=U0_min):
            U0_horizon = min(U0_max-curr_t, horizon)
        else:
            U0_horizon = min(curr_t+horizon, U0_max)-U0_min
            U0_horizon = min(horizon, U0_horizon)
        start_time_U0 = max(0, U0_min-curr_t)
        rho_U0 = model.addVar(lb = None, ub = None, vtype="C", name="rho_U0")
        rho_U0_temp = model.addVar(lb = None, ub = None, vtype="C", name="rho_U0")
        for i in range(U0_horizon + 1):
            for p in range(2):
                rho_U0_1[i, p] = model.addVar(lb = None, ub = None, vtype="C", name="rho_U0_1")
                rho_U0_2[i, p] = model.addVar(lb = None, ub = None, vtype="C", name="rho_U0_2")
            U_01[i] = model.addVar(lb = None, ub = None, vtype="C", name="U_01")
            U_02[i] = model.addVar(lb = None, ub = None, vtype="C", name="U_02")
            U_03[i] = model.addVar(lb = None, ub = None, vtype="C", name="U_03")
            U_0[i] = model.addVar(lb = None, ub = None, vtype="C", name="U_0")
        for i in range(U0_horizon+1):
            # print(i+start_time_U0)
            model.addCons(rho_U0_1[i, 0] == z[i+start_time_U0, 0] - A3_1[0])
            model.addCons(rho_U0_1[i, 1] == A3_1[1] - z[i+start_time_U0, 0])
            model.addCons(rho_U0_2[i, 0] == z[i+start_time_U0, 0] - A3_2[0])
            model.addCons(rho_U0_2[i, 1] == A3_2[1] - z[i+start_time_U0, 0])    
            addConsmymin(model, [rho_U0_1[i, p] for p in range(2)], U_01[i])
            addConsmymin(model, [rho_U0_2[i, p] for p in range(2)], U_02[i]) 
        addConsmymin(model, [U_01[i] for i in range(U0_horizon + 1)], rho_U0_temp)
        for i in range(U0_horizon+1):
            addConsmymin(model, [U_01[j] for j in range(i+1)], U_03[i])
            addConsmymin(model, [U_03[i], U_02[i]], U_0[i])
        addConsmymax(model, [U_0[i] for i in range(U0_horizon + 1)], rho_U0)
        rho_U0_t = model.addVar(lb = None, ub = None, vtype="C", name="rho_U0_t")
        addConsmymax(model, [rho_U0_temp, rho_U0], rho_U0_t)
   
    consider_lst = [rho_physical]
    obj = model.addVar(lb = None, ub = None, vtype="C", name="rho")
    
    if(curr_t <= F0_max and 1 in I):
        consider_lst.append(rho_F0)
    if(curr_t <= G0_max and 2 in I):
        consider_lst.append(rho_G0)  
    if(curr_t+horizon >= U0_min and curr_t+horizon < U0_max and 3 in I):
        consider_lst.append(rho_U0_t)
    if(curr_t+horizon>=U0_max and 3 in I):
        consider_lst.append(rho_U0)
    print("I is :{}".format(I))
    for i in range(len(consider_lst)):
        t[i] = model.addVar(lb = None, ub = None, vtype="C", name="t")
        model.addCons(t[i] == consider_lst[i])
    t[len(consider_lst)] = model.addVar(lb = None, ub = None, vtype="C", name="t")
    model.addCons(t[len(consider_lst)] == obj)
    addConsmymin(model, consider_lst, obj)
    model.addCons(obj >= 0.15)

def Add_obj(model,u,horizon):
    obj = model.addVar(vtype = "C", name="obj")
    model.addCons(obj == sum(u[i, 0]**2 for i in range(horizon)))                                                                                                                                               
    model.setObjective(obj, "minimize")
    return obj


def update(curr_s, u, state_sequence):
    temp = [curr_s[0], curr_s[1]]
    for i in range(len(u)):
        temp[0] += (0.5*temp[1]+0.5*u[i]+Disturb_max * 0.1 * random.randint(-10, 10))
        temp[1] += (u[i]+Disturb_max * 0.1 * random.randint(-10, 10))
        state_sequence.append([temp[0]])
    print("updating: ", curr_s, u, temp)
    return temp

# start = time.time()
# predict(0,[6, 0], 20)
# end = time.time()
# print(end-start)