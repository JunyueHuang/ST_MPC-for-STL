using JLD
using LazySets
using ReachabilityBase
include("parameter.jl")
include("forward.jl")
function onestepset1(X, U, X_next)
    temp=[];
    X_next_tmp = X_next
    X_previous = convert(HPolytope, inv(A)*intersection(X, minkowski_sum(X_next_tmp, convert(Zonotope,B*U))));
    if X_previous.constraints != []
        push!(temp, intersection(X, X_previous));
    end
    results = UnionSetArray([temp[i] for i in 1: length(temp)]);
    return results
end

function onestepset(X, U, X_next::EmptySet)
    return LazySets.∅(2)
end

function onestepset(X, U, X_next)
    temp=[];
    for i = 1:length(X_next)
            X_next_tmp = convert(HPolytope, X_next[i])
            X_previous = convert(HPolytope, inv(A)*intersection(X, minkowski_sum(X_next_tmp, convert(Zonotope,B*U))));
            if X_previous.constraints != []
                push!(temp, intersection(X, X_previous));
            end
      end
    results = UnionSetArray([temp[i] for i in 1: length(temp)]);
    return results
end


if !isfile("offline.jld")
    #X^I_t 
    BigX = [];
    tempset = [];
    push!(BigX,UnionSetArray([convert(HPolytope,goal[4])]))# Terminal X^I_t

    tempset = onestepset1(convert(HPolytope,goal[3]),U,convert(HPolytope,goal[4]))
    tempset = UnionSetArray([convert(HPolytope,convex_hull(tempset[1],goal[4]))])
    pushfirst!(BigX,tempset) #19th
    #12-18
    for i in 1:7
        global tempset=onestepset(convert(HPolytope,goal[3]),U,tempset)
        global tempset = UnionSetArray([convert(HPolytope,convex_hull(tempset[1],goal[4]))])
        pushfirst!(BigX,tempset)
    end

    #9-11
    for i in 1:3
        global tempset=onestepset(convert(HPolytope,X),U,tempset)
        pushfirst!(BigX,tempset)
    end

    #0-8
    for i in 1:9
        global tempset=onestepset(convert(HPolytope,goal[2]),U,tempset)
        pushfirst!(BigX,tempset)
    end
    #0-8 F
    BigX1=[];
    push!(BigX1,UnionSetArray([convert(HPolytope,intersection(BigX[9],goal[1]))]));
    tempset1 = onestepset1(convert(HPolytope,goal[2]),U,convert(HPolytope,goal[1]));
    tempset1 = UnionSetArray([convert(HPolytope,intersection(BigX[8],convex_hull(tempset1[1],goal[1])))]);
    pushfirst!(BigX1,tempset1)
    for i in 1:7
        global tempset1 = onestepset(convert(HPolytope,goal[2]),U,tempset1);
        global tempset1 = UnionSetArray([convert(HPolytope,intersection(BigX[8-i],convex_hull(tempset[1],goal[1])))]);
        pushfirst!(BigX1,tempset1);
    end

    save("offline.jld", "BigX", BigX, "BigX1", BigX1)
else
    # 加载数据
    loaded_data = load("offline.jld")
    BigX = loaded_data["BigX"]
    BigX1 = loaded_data["BigX1"]
end

# return backward feasible sets at time t of I-remaining formula 
function feasible_set(t,i1,i3)
    X_fs = convert(HPolytope,X)
    if t <=endt[1]
        if i1 == 1
            X_fs = intersection(X_fs,BigX1[t][1])
        else
            X_fs = intersection(X_fs,BigX[t][1])
        end
    end
    X_fs = BigX[t][1]
    if t >=begint[2]
        if i3 == 1
            X_fs = intersection(X_fs,BigX[t][1])
        else
            X_fs = convert(HPolytope,X)
        end
    end
    return X_fs
end
