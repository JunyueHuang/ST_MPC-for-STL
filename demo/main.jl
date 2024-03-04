using IntervalArithmetic
using LazySets
using Plots
using Polyhedra, CDDLib
using LaTeXStrings
using JLD, HDF5
include("forward.jl")
include("offline_set.jl")
include("recomp.jl")
include("parameter.jl")
# I1, I3 = 0, 0;

function trigger(z, vz, u::Vector{Float64}, t, I1, I3)
    # println(I1, I3)
    println("running trigger-time while I1 = $I1, I3 = $I3")    
    x_state = [z,vz];
    fs = feasible_set(t,I1,I3);
    if (x_state ∈ fs) == false
        println("violated");
        return 1
    end
    #update Ix
    if (x_state ∈ goal[1])&& t<=endt[1]
        I1 = 0;
    end
    if (x_state ∈ intersection(goal[3],goal[4]))&& t>=begint[2]
        I3 = 0;
    end
    if !isfile("augumented.jld")
        save("augumented.jld","aug",0)
    end

    step = 1; k = 1;
    beliefx = forwardset1(z,vz,u[step]);
    if issubset(beliefx,feasible_set(t+1,I1,I3))==false
        return 1
    end
    Augset = [];# belief state at t+1
    for i in 1:length(Fhat(augmented_state(beliefx,I1,I3),t+1))
        push!(Augset,Fhat(augmented_state(beliefx,I1,I3),t+1)[i]); # t+1
    end
    while step <= min(5, length(u)) # maxtime
        println("trying trigger time $step with length of u = $(length(u))")
        if determinecheck(Augset)
            k = step;
            rm("augumented.jld")
            save("augumented.jld","aug",Augset)
        end
        if step+1 > length(u)
            return k
        end
        Augset = generate_newbelief(Augset, u[step+1],t+step+1) #belief state at t+step+1
        if Augset == false
            return k
        else
            step += 1
        end
    end
    return k
end

function refine(curr_s, I1, I3)
    # for b = {math.x, I1', I3'} in Augset, if z \in math.x, then I1=I1', I3=I3'
    loaded_data = load("augumented.jld")
    Augset = loaded_data["aug"] 
    for i in 1:length(Augset)
        println(Augset[i].mathx)
        println("\n")
        if issubset(Singleton(curr_s), Augset[i].mathx)
            I1 = Augset[i].I1
            I3 = Augset[i].I3
            return I1, I3
        end
    end 
    println("not found")
    return I1, I3
end
# println(main_function(endt, z, vz))
# for i in 1:length(Augset)
#     plot!(Augset[i].mathx)
# end