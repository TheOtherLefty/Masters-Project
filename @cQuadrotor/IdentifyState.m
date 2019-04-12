function [state, flag] = IdentifyState(obj, Y, SearchStatus)

    % Identifies current state of simulation
    % Current position and grid point
    x = round(Y(1)/obj.CellSize);
    y = round(Y(2)/obj.CellSize);
    gp = y*obj.GridSize(1) + x;
    obj.Decisions.Gridpoints(1) = 0; % In new model base is always set as searched.
    
    % Num of targets to find
    objs = obj.NumTargets - obj.TargetCount;
    
    % Find current state in table
    tol = 1e-6;
    
    cond{1} = abs(obj.Decisions.States.posx - x) < tol;
    cond{2} = abs(obj.Decisions.States.posy - y) < tol;
    cond{3} = abs(obj.Decisions.States.objs - objs) < tol;
    
    cond{4} = abs(obj.Decisions.States.gps(:,1) - obj.Decisions.Gridpoints(:,1)) < tol;
    cond{5} = abs(obj.Decisions.States.gps(:,2) - obj.Decisions.Gridpoints(:,2)) < tol;
    cond{6} = abs(obj.Decisions.States.gps(:,3) - obj.Decisions.Gridpoints(:,3)) < tol;
    cond{7} = abs(obj.Decisions.States.gps(:,4) - obj.Decisions.Gridpoints(:,4)) < tol;
    cond{8} = abs(obj.Decisions.States.gps(:,5) - obj.Decisions.Gridpoints(:,5)) < tol;
    cond{9} = abs(obj.Decisions.States.gps(:,6) - obj.Decisions.Gridpoints(:,6)) < tol;
    cond{10} = abs(obj.Decisions.States.gps(:,7) - obj.Decisions.Gridpoints(:,7)) < tol;
    cond{11} = abs(obj.Decisions.States.gps(:,8) - obj.Decisions.Gridpoints(:,8)) < tol;
    cond{12} = abs(obj.Decisions.States.gps(:,9) - obj.Decisions.Gridpoints(:,9)) < tol;
    cond{13} = abs(obj.Decisions.States.gps(:,10) - obj.Decisions.Gridpoints(:,10)) < tol;
    cond{14} = abs(obj.Decisions.States.gps(:,11) - obj.Decisions.Gridpoints(:,11)) < tol;
    cond{15} = abs(obj.Decisions.States.gps(:,12) - obj.Decisions.Gridpoints(:,12)) < tol;
    cond{16} = abs(obj.Decisions.States.gps(:,13) - obj.Decisions.Gridpoints(:,13)) < tol;
    cond{17} = abs(obj.Decisions.States.gps(:,14) - obj.Decisions.Gridpoints(:,14)) < tol;
    cond{18} = abs(obj.Decisions.States.gps(:,15) - obj.Decisions.Gridpoints(:,15)) < tol;
    cond{19} = abs(obj.Decisions.States.gps(:,16) - obj.Decisions.Gridpoints(:,16)) < tol;
    cond{20} = abs(obj.Decisions.States.gps(:,17) - obj.Decisions.Gridpoints(:,17)) < tol;
    cond{21} = abs(obj.Decisions.States.gps(:,18) - obj.Decisions.Gridpoints(:,18)) < tol;
    cond{22} = abs(obj.Decisions.States.gps(:,19) - obj.Decisions.Gridpoints(:,19)) < tol;
    cond{23} = abs(obj.Decisions.States.gps(:,20) - obj.Decisions.Gridpoints(:,20)) < tol;
    cond{24} = abs(obj.Decisions.States.gps(:,21) - obj.Decisions.Gridpoints(:,21)) < tol;
    cond{25} = abs(obj.Decisions.States.gps(:,22) - obj.Decisions.Gridpoints(:,22)) < tol;
    cond{26} = abs(obj.Decisions.States.gps(:,23) - obj.Decisions.Gridpoints(:,23)) < tol;
    cond{27} = abs(obj.Decisions.States.gps(:,24) - obj.Decisions.Gridpoints(:,24)) < tol;
    cond{28} = abs(obj.Decisions.States.gps(:,25) - obj.Decisions.Gridpoints(:,25)) < tol;
    cond{29} = abs(obj.Decisions.States.b - obj.BatteryLevel) < tol;
    
    cond{30} = abs(obj.Decisions.States.s - SearchStatus) < tol;
    
    conditions = cond{1} & cond{2} & cond{3} & cond{4} & cond{5} & cond{6} & cond{7} & cond{8} & cond{9} & cond{10} & cond{11} & cond{12} & cond{13} & cond{14} & cond{15} & cond{16} & cond{17} & cond{18} & cond{19} & cond{20} & cond{21} & cond{22} & cond{23} & cond{24} & cond{25} & cond{26} & cond{27} & cond{28} & cond{29} & cond{30};
    
    state = obj.Decisions.States.state(conditions);
    
    % fprintf('Current state values: posx: %d, posy: %d, objs: %d, b: %d\n', x, y, objs, obj.BatteryLevel)
    % fprintf('    gps = [ '), fprintf('%d ', obj.Decisions.Gridpoints), fprintf(']\n\n')
    
    if isempty(state)
        flag = 1;
    else
        flag = 0;
    end
end


