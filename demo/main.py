from func import *
from Parameter import *
from julia import Main
import time

Main.include("main.jl")
I1, I3 = 1, 1
start = 0
curr_s = [6, 0]
t1 = time.time()  
state_sequence = [[curr_s[0]]]
trigger_sequence = [start]
while start < time_max:
    print("current state: ", curr_s, "current time: ", start)
    if(1 in I and curr_s[0]>=A1[0] and curr_s[0]<=A1[1]):
        I1 = 0
        I.remove(1)
    if I1 == 0 and 1 in I:I.remove(1)
    if(2 in I and start > G0_max):
        I.remove(2) 
    if(3 in I and start>=U0_min and start<=U0_max and curr_s[0]>=A3_2[0] and curr_s[0]<=A3_2[1]):
        I3 = 0
        I.remove(3)
    if I3 == 0 and 3 in I:I.remove(3)
    u = predict(start, curr_s, time_max-start)
    tau= Main.trigger(curr_s[0], curr_s[1], u, start+1, I1, I3)
    u = u[:tau]
    curr_s = update(curr_s, u, state_sequence)  
    I1, I3 = Main.refine(curr_s, I1, I3)
    start = start+tau
    trigger_sequence.append(start)
t2 = time.time()    
print("time: ", t2-t1)  
printSolution(state_sequence, trigger_sequence)