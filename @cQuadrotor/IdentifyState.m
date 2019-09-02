function [state, flag] = IdentifyState(obj, Y, SearchStatus)

    % Identifies current state of simulation
    % Current position
    [x,y] = adjustCoords(obj,round(Y(1)/obj.CellSize),round(Y(2)/obj.CellSize));
    
    x = abs(x);
    y = abs(y);
    
    % As objects won't spawn at base, it is assumed that these are searched
    % already.
    obj.Decisions.Gridpoints(55) = 0;
    obj.Decisions.Gridpoints(56) = 0;
    obj.Decisions.Gridpoints(45) = 0;
    obj.Decisions.Gridpoints(46) = 0;
    
    % Num of targets to find
    objs = obj.NumTargets - obj.TargetCount;
    
    % Find current state in table
    tol = 1e-6;
    
    % Translates 10*10 list of gridpoints into a 5*5 list of gridpoints for
    % the current quadrant.
    quadrantGPS = translateGPS(obj);
    
    cond{1} = abs(obj.Decisions.States.posx - x) < tol;
    cond{2} = abs(obj.Decisions.States.posy - y) < tol;
    cond{3} = abs(obj.Decisions.States.objs - objs) < tol;
    
    cond{4} = abs(obj.Decisions.States.gps(:,1) - quadrantGPS(:,1)) < tol;
    cond{5} = abs(obj.Decisions.States.gps(:,2) - quadrantGPS(:,2)) < tol;
    cond{6} = abs(obj.Decisions.States.gps(:,3) - quadrantGPS(:,3)) < tol;
    cond{7} = abs(obj.Decisions.States.gps(:,4) - quadrantGPS(:,4)) < tol;
    cond{8} = abs(obj.Decisions.States.gps(:,5) - quadrantGPS(:,5)) < tol;
    cond{9} = abs(obj.Decisions.States.gps(:,6) - quadrantGPS(:,6)) < tol;
    cond{10} = abs(obj.Decisions.States.gps(:,7) - quadrantGPS(:,7)) < tol;
    cond{11} = abs(obj.Decisions.States.gps(:,8) - quadrantGPS(:,8)) < tol;
    cond{12} = abs(obj.Decisions.States.gps(:,9) - quadrantGPS(:,9)) < tol;
    cond{13} = abs(obj.Decisions.States.gps(:,10) - quadrantGPS(:,10)) < tol;
    cond{14} = abs(obj.Decisions.States.gps(:,11) - quadrantGPS(:,11)) < tol;
    cond{15} = abs(obj.Decisions.States.gps(:,12) - quadrantGPS(:,12)) < tol;
    cond{16} = abs(obj.Decisions.States.gps(:,13) - quadrantGPS(:,13)) < tol;
    cond{17} = abs(obj.Decisions.States.gps(:,14) - quadrantGPS(:,14)) < tol;
    cond{18} = abs(obj.Decisions.States.gps(:,15) - quadrantGPS(:,15)) < tol;
    cond{19} = abs(obj.Decisions.States.gps(:,16) - quadrantGPS(:,16)) < tol;
    cond{20} = abs(obj.Decisions.States.gps(:,17) - quadrantGPS(:,17)) < tol;
    cond{21} = abs(obj.Decisions.States.gps(:,18) - quadrantGPS(:,18)) < tol;
    cond{22} = abs(obj.Decisions.States.gps(:,19) - quadrantGPS(:,19)) < tol;
    cond{23} = abs(obj.Decisions.States.gps(:,20) - quadrantGPS(:,20)) < tol;
    cond{24} = abs(obj.Decisions.States.gps(:,21) - quadrantGPS(:,21)) < tol;
    cond{25} = abs(obj.Decisions.States.gps(:,22) - quadrantGPS(:,22)) < tol;
    cond{26} = abs(obj.Decisions.States.gps(:,23) - quadrantGPS(:,23)) < tol;
    cond{27} = abs(obj.Decisions.States.gps(:,24) - quadrantGPS(:,24)) < tol;
    cond{28} = abs(obj.Decisions.States.gps(:,25) - quadrantGPS(:,25)) < tol;
    cond{29} = abs(obj.Decisions.States.b - obj.BatteryLevel) < tol;
    
    cond{30} = abs(obj.Decisions.States.s - SearchStatus) < tol;
    
    conditions = cond{1} & cond{2} & cond{3} & cond{4} & cond{5} & cond{6} & cond{7} & cond{8} & cond{9} & cond{10} & cond{11} & cond{12} & cond{13} & cond{14} & cond{15} & cond{16} & cond{17} & cond{18} & cond{19} & cond{20} & cond{21} & cond{22} & cond{23} & cond{24} & cond{25} & cond{26} & cond{27} & cond{28} & cond{29} & cond{30};
    
    state = obj.Decisions.States.state(conditions);
    
    %fprintf('\n')
    %fprintf('Current state values: s: %d, q: %d, posx: %d, posy: %d, objs: %d, b: %d\n', SearchStatus, obj.SearchedQuadrants, x, y, objs, obj.BatteryLevel)
    %fprintf('    gps = [ '), fprintf('%d ', quadrantGPS), fprintf(']\n')
    
    if isempty(state)
        flag = 1;
        
        if obj.BatteryLevel ~= 32
            % Debugging output
            condx = cond{2} & cond{3} & cond{4} & cond{5} & cond{6} & cond{7} & cond{8} & cond{9} & cond{10} & cond{11} & cond{12} & cond{13} & cond{14} & cond{15} & cond{16} & cond{17} & cond{18} & cond{19} & cond{20} & cond{21} & cond{22} & cond{23} & cond{24} & cond{25} & cond{26} & cond{27} & cond{28} & cond{29} & cond{30};
            condy = cond{2} & cond{3} & cond{4} & cond{5} & cond{6} & cond{7} & cond{8} & cond{9} & cond{10} & cond{11} & cond{12} & cond{13} & cond{14} & cond{15} & cond{16} & cond{17} & cond{18} & cond{19} & cond{20} & cond{21} & cond{22} & cond{23} & cond{24} & cond{25} & cond{26} & cond{27} & cond{28} & cond{29} & cond{30};
            condb = cond{2} & cond{3} & cond{4} & cond{5} & cond{6} & cond{7} & cond{8} & cond{9} & cond{10} & cond{11} & cond{12} & cond{13} & cond{14} & cond{15} & cond{16} & cond{17} & cond{18} & cond{19} & cond{20} & cond{21} & cond{22} & cond{23} & cond{24} & cond{25} & cond{26} & cond{27} & cond{28} & cond{1} & cond{30};
            conds = cond{2} & cond{3} & cond{4} & cond{5} & cond{6} & cond{7} & cond{8} & cond{9} & cond{10} & cond{11} & cond{12} & cond{13} & cond{14} & cond{15} & cond{16} & cond{17} & cond{18} & cond{19} & cond{20} & cond{21} & cond{22} & cond{23} & cond{24} & cond{25} & cond{26} & cond{27} & cond{28} & cond{29} & cond{1};
            condobjs = cond{2} & cond{1} & cond{4} & cond{5} & cond{6} & cond{7} & cond{8} & cond{9} & cond{10} & cond{11} & cond{12} & cond{13} & cond{14} & cond{15} & cond{16} & cond{17} & cond{18} & cond{19} & cond{20} & cond{21} & cond{22} & cond{23} & cond{24} & cond{25} & cond{26} & cond{27} & cond{28} & cond{29} & cond{30};
            cond4 = cond{1} & cond{2} & cond{3} & cond{5} & cond{6} & cond{7} & cond{8} & cond{9} & cond{10} & cond{11} & cond{12} & cond{13} & cond{14} & cond{15} & cond{16} & cond{17} & cond{18} & cond{19} & cond{20} & cond{21} & cond{22} & cond{23} & cond{24} & cond{25} & cond{26} & cond{27} & cond{28} & cond{29} & cond{30};
            cond5 = cond{1} & cond{2} & cond{3} & cond{4} & cond{6} & cond{7} & cond{8} & cond{9} & cond{10} & cond{11} & cond{12} & cond{13} & cond{14} & cond{15} & cond{16} & cond{17} & cond{18} & cond{19} & cond{20} & cond{21} & cond{22} & cond{23} & cond{24} & cond{25} & cond{26} & cond{27} & cond{28} & cond{29} & cond{30};
            cond6 = cond{1} & cond{2} & cond{3} & cond{4} & cond{5} & cond{7} & cond{8} & cond{9} & cond{10} & cond{11} & cond{12} & cond{13} & cond{14} & cond{15} & cond{16} & cond{17} & cond{18} & cond{19} & cond{20} & cond{21} & cond{22} & cond{23} & cond{24} & cond{25} & cond{26} & cond{27} & cond{28} & cond{29} & cond{30};
            cond7 = cond{1} & cond{2} & cond{3} & cond{4} & cond{5} & cond{6} & cond{8} & cond{9} & cond{10} & cond{11} & cond{12} & cond{13} & cond{14} & cond{15} & cond{16} & cond{17} & cond{18} & cond{19} & cond{20} & cond{21} & cond{22} & cond{23} & cond{24} & cond{25} & cond{26} & cond{27} & cond{28} & cond{29} & cond{30};
            cond8 = cond{1} & cond{2} & cond{3} & cond{4} & cond{5} & cond{6} & cond{7} & cond{9} & cond{10} & cond{11} & cond{12} & cond{13} & cond{14} & cond{15} & cond{16} & cond{17} & cond{18} & cond{19} & cond{20} & cond{21} & cond{22} & cond{23} & cond{24} & cond{25} & cond{26} & cond{27} & cond{28} & cond{29} & cond{30};
            cond9 = cond{1} & cond{2} & cond{3} & cond{4} & cond{5} & cond{6} & cond{7} & cond{8} & cond{10} & cond{11} & cond{12} & cond{13} & cond{14} & cond{15} & cond{16} & cond{17} & cond{18} & cond{19} & cond{20} & cond{21} & cond{22} & cond{23} & cond{24} & cond{25} & cond{26} & cond{27} & cond{28} & cond{29} & cond{30};
            cond10 = cond{1} & cond{2} & cond{3} & cond{4} & cond{5} & cond{6} & cond{7} & cond{8} & cond{9} & cond{11} & cond{12} & cond{13} & cond{14} & cond{15} & cond{16} & cond{17} & cond{18} & cond{19} & cond{20} & cond{21} & cond{22} & cond{23} & cond{24} & cond{25} & cond{26} & cond{27} & cond{28} & cond{29} & cond{30};
            cond11 = cond{1} & cond{2} & cond{3} & cond{4} & cond{5} & cond{6} & cond{7} & cond{8} & cond{9} & cond{10} & cond{12} & cond{13} & cond{14} & cond{15} & cond{16} & cond{17} & cond{18} & cond{19} & cond{20} & cond{21} & cond{22} & cond{23} & cond{24} & cond{25} & cond{26} & cond{27} & cond{28} & cond{29} & cond{30};
            cond12 = cond{1} & cond{2} & cond{3} & cond{4} & cond{5} & cond{6} & cond{7} & cond{8} & cond{9} & cond{10} & cond{11} & cond{13} & cond{14} & cond{15} & cond{16} & cond{17} & cond{18} & cond{19} & cond{20} & cond{21} & cond{22} & cond{23} & cond{24} & cond{25} & cond{26} & cond{27} & cond{28} & cond{29} & cond{30};
            cond13 = cond{1} & cond{2} & cond{3} & cond{4} & cond{5} & cond{6} & cond{7} & cond{8} & cond{9} & cond{10} & cond{11} & cond{12} & cond{14} & cond{15} & cond{16} & cond{17} & cond{18} & cond{19} & cond{20} & cond{21} & cond{22} & cond{23} & cond{24} & cond{25} & cond{26} & cond{27} & cond{28} & cond{29} & cond{30};
            cond14 = cond{1} & cond{2} & cond{3} & cond{4} & cond{5} & cond{6} & cond{7} & cond{8} & cond{9} & cond{10} & cond{11} & cond{12} & cond{13} & cond{15} & cond{16} & cond{17} & cond{18} & cond{19} & cond{20} & cond{21} & cond{22} & cond{23} & cond{24} & cond{25} & cond{26} & cond{27} & cond{28} & cond{29} & cond{30};
            cond15 = cond{1} & cond{2} & cond{3} & cond{4} & cond{5} & cond{6} & cond{7} & cond{8} & cond{9} & cond{10} & cond{11} & cond{12} & cond{13} & cond{14} & cond{16} & cond{17} & cond{18} & cond{19} & cond{20} & cond{21} & cond{22} & cond{23} & cond{24} & cond{25} & cond{26} & cond{27} & cond{28} & cond{29} & cond{30};
            cond16 = cond{1} & cond{2} & cond{3} & cond{4} & cond{5} & cond{6} & cond{7} & cond{8} & cond{9} & cond{10} & cond{11} & cond{12} & cond{13} & cond{14} & cond{15} & cond{17} & cond{18} & cond{19} & cond{20} & cond{21} & cond{22} & cond{23} & cond{24} & cond{25} & cond{26} & cond{27} & cond{28} & cond{29} & cond{30};
            cond17 = cond{1} & cond{2} & cond{3} & cond{4} & cond{5} & cond{6} & cond{7} & cond{8} & cond{9} & cond{10} & cond{11} & cond{12} & cond{13} & cond{14} & cond{15} & cond{16} & cond{18} & cond{19} & cond{20} & cond{21} & cond{22} & cond{23} & cond{24} & cond{25} & cond{26} & cond{27} & cond{28} & cond{29} & cond{30};
            cond18 = cond{1} & cond{2} & cond{3} & cond{4} & cond{5} & cond{6} & cond{7} & cond{8} & cond{9} & cond{10} & cond{11} & cond{12} & cond{13} & cond{14} & cond{15} & cond{16} & cond{17} & cond{19} & cond{20} & cond{21} & cond{22} & cond{23} & cond{24} & cond{25} & cond{26} & cond{27} & cond{28} & cond{29} & cond{30};
            cond19 = cond{1} & cond{2} & cond{3} & cond{4} & cond{5} & cond{6} & cond{7} & cond{8} & cond{9} & cond{10} & cond{11} & cond{12} & cond{13} & cond{14} & cond{15} & cond{16} & cond{17} & cond{18} & cond{20} & cond{21} & cond{22} & cond{23} & cond{24} & cond{25} & cond{26} & cond{27} & cond{28} & cond{29} & cond{30};
            cond20 = cond{1} & cond{2} & cond{3} & cond{4} & cond{5} & cond{6} & cond{7} & cond{8} & cond{9} & cond{10} & cond{11} & cond{12} & cond{13} & cond{14} & cond{15} & cond{16} & cond{17} & cond{18} & cond{19} & cond{21} & cond{22} & cond{23} & cond{24} & cond{25} & cond{26} & cond{27} & cond{28} & cond{29} & cond{30};
            cond21 = cond{1} & cond{2} & cond{3} & cond{4} & cond{5} & cond{6} & cond{7} & cond{8} & cond{9} & cond{10} & cond{11} & cond{12} & cond{13} & cond{14} & cond{15} & cond{16} & cond{17} & cond{18} & cond{19} & cond{20} & cond{22} & cond{23} & cond{24} & cond{25} & cond{26} & cond{27} & cond{28} & cond{29} & cond{30};
            cond22 = cond{1} & cond{2} & cond{3} & cond{4} & cond{5} & cond{6} & cond{7} & cond{8} & cond{9} & cond{10} & cond{11} & cond{12} & cond{13} & cond{14} & cond{15} & cond{16} & cond{17} & cond{18} & cond{19} & cond{20} & cond{21} & cond{23} & cond{24} & cond{25} & cond{26} & cond{27} & cond{28} & cond{29} & cond{30};
            cond23 = cond{1} & cond{2} & cond{3} & cond{4} & cond{5} & cond{6} & cond{7} & cond{8} & cond{9} & cond{10} & cond{11} & cond{12} & cond{13} & cond{14} & cond{15} & cond{16} & cond{17} & cond{18} & cond{19} & cond{20} & cond{21} & cond{22} & cond{24} & cond{25} & cond{26} & cond{27} & cond{28} & cond{29} & cond{30};
            cond24 = cond{1} & cond{2} & cond{3} & cond{4} & cond{5} & cond{6} & cond{7} & cond{8} & cond{9} & cond{10} & cond{11} & cond{12} & cond{13} & cond{14} & cond{15} & cond{16} & cond{17} & cond{18} & cond{19} & cond{20} & cond{21} & cond{22} & cond{23} & cond{25} & cond{26} & cond{27} & cond{28} & cond{29} & cond{30};
            cond25 = cond{1} & cond{2} & cond{3} & cond{4} & cond{5} & cond{6} & cond{7} & cond{8} & cond{9} & cond{10} & cond{11} & cond{12} & cond{13} & cond{14} & cond{15} & cond{16} & cond{17} & cond{18} & cond{19} & cond{20} & cond{21} & cond{22} & cond{23} & cond{24} & cond{26} & cond{27} & cond{28} & cond{29} & cond{30};
            cond26 = cond{1} & cond{2} & cond{3} & cond{4} & cond{5} & cond{6} & cond{7} & cond{8} & cond{9} & cond{10} & cond{11} & cond{12} & cond{13} & cond{14} & cond{15} & cond{16} & cond{17} & cond{18} & cond{19} & cond{20} & cond{21} & cond{22} & cond{23} & cond{24} & cond{25} & cond{27} & cond{28} & cond{29} & cond{30};
            cond27 = cond{1} & cond{2} & cond{3} & cond{4} & cond{5} & cond{6} & cond{7} & cond{8} & cond{9} & cond{10} & cond{11} & cond{12} & cond{13} & cond{14} & cond{15} & cond{16} & cond{17} & cond{18} & cond{19} & cond{20} & cond{21} & cond{22} & cond{23} & cond{24} & cond{25} & cond{26} & cond{28} & cond{29} & cond{30};
            cond28 = cond{1} & cond{2} & cond{3} & cond{4} & cond{5} & cond{6} & cond{7} & cond{8} & cond{9} & cond{10} & cond{11} & cond{12} & cond{13} & cond{14} & cond{15} & cond{16} & cond{17} & cond{18} & cond{19} & cond{20} & cond{21} & cond{22} & cond{23} & cond{24} & cond{25} & cond{26} & cond{27} & cond{29} & cond{30};
            condxy = cond{3} & cond{4} & cond{5} & cond{6} & cond{7} & cond{8} & cond{9} & cond{10} & cond{11} & cond{12} & cond{13} & cond{14} & cond{15} & cond{16} & cond{17} & cond{18} & cond{19} & cond{20} & cond{21} & cond{22} & cond{23} & cond{24} & cond{25} & cond{26} & cond{27} & cond{28} & cond{29} & cond{30};
            condnogps = cond{3} & cond{29} & cond{30} & cond{1} & cond{2};
            condonlygps = cond{4} & cond{5} & cond{6} & cond{7} & cond{8} & cond{9} & cond{10} & cond{11} & cond{12} & cond{13} & cond{14} & cond{15} & cond{16} & cond{17} & cond{18} & cond{19} & cond{20} & cond{21} & cond{22} & cond{23} & cond{24} & cond{25} & cond{26} & cond{27} & cond{28};
            cond1to5 = cond{6} & cond{7} & cond{8} & cond{9} & cond{10} & cond{11} & cond{12} & cond{13} & cond{14} & cond{15} & cond{16} & cond{17} & cond{18} & cond{19} & cond{20} & cond{21} & cond{22} & cond{23} & cond{24} & cond{25} & cond{26} & cond{27} & cond{28} & cond{29} & cond{30};
            cond6to10 = cond{1} & cond{2} & cond{3} & cond{4} & cond{5} & cond{11} & cond{12} & cond{13} & cond{14} & cond{15} & cond{16} & cond{17} & cond{18} & cond{19} & cond{20} & cond{21} & cond{22} & cond{23} & cond{24} & cond{25} & cond{26} & cond{27} & cond{28} & cond{29} & cond{30};
            cond11to15 = cond{1} & cond{2} & cond{3} & cond{4} & cond{5} & cond{6} & cond{7} & cond{8} & cond{9} & cond{10} & cond{16} & cond{17} & cond{18} & cond{19} & cond{20} & cond{21} & cond{22} & cond{23} & cond{24} & cond{25} & cond{26} & cond{27} & cond{28} & cond{29} & cond{30};
            cond16to20 = cond{1} & cond{2} & cond{3} & cond{4} & cond{5} & cond{6} & cond{7} & cond{8} & cond{9} & cond{10} & cond{11} & cond{12} & cond{13} & cond{14} & cond{15} & cond{21} & cond{22} & cond{23} & cond{24} & cond{25} & cond{26} & cond{27} & cond{28} & cond{29} & cond{30};
            cond21to25 = cond{1} & cond{2} & cond{3} & cond{4} & cond{5} & cond{6} & cond{7} & cond{8} & cond{9} & cond{10} & cond{11} & cond{12} & cond{13} & cond{14} & cond{15} & cond{16} & cond{17} & cond{18} & cond{19} & cond{20} & cond{26} & cond{27} & cond{28} & cond{29} & cond{30};
            cond26to30 = cond{1} & cond{2} & cond{3} & cond{4} & cond{5} & cond{6} & cond{7} & cond{8} & cond{9} & cond{10} & cond{11} & cond{12} & cond{13} & cond{14} & cond{15} & cond{16} & cond{17} & cond{18} & cond{19} & cond{20} & cond{21} & cond{22} & cond{23} & cond{24} & cond{25};
            cond1to15 =  cond{16} & cond{17} & cond{18} & cond{19} & cond{20} & cond{21} & cond{22} & cond{23} & cond{24} & cond{25} & cond{26} & cond{27} & cond{28} & cond{29} & cond{30};
            cond16to30 = cond{1} & cond{2} & cond{3} & cond{4} & cond{5} & cond{6} & cond{7} & cond{8} & cond{9} & cond{10} & cond{11} & cond{12} & cond{13} & cond{14} & cond{15};

            statesxy = obj.Decisions.States.state(condxy)    
            statesx = obj.Decisions.States.state(condx)
            statesy = obj.Decisions.States.state(condy)
            statesb = obj.Decisions.States.state(condb)
            statess = obj.Decisions.States.state(conds)
            statesobjs = obj.Decisions.States.state(condobjs)
            states4 = obj.Decisions.States.state(cond4)
            states5 = obj.Decisions.States.state(cond5)
            states6 = obj.Decisions.States.state(cond6)
            states7 = obj.Decisions.States.state(cond7)
            states8 = obj.Decisions.States.state(cond8)
            states9 = obj.Decisions.States.state(cond9)
            states10 = obj.Decisions.States.state(cond10)
            states11 = obj.Decisions.States.state(cond11)
            states12 = obj.Decisions.States.state(cond12)
            states13 = obj.Decisions.States.state(cond13)
            states14 = obj.Decisions.States.state(cond14)
            states15 = obj.Decisions.States.state(cond15)
            states16 = obj.Decisions.States.state(cond16)
            states17 = obj.Decisions.States.state(cond17)
            states18 = obj.Decisions.States.state(cond18)
            states19 = obj.Decisions.States.state(cond19)
            states20 = obj.Decisions.States.state(cond20)
            states21 = obj.Decisions.States.state(cond21)
            states22 = obj.Decisions.States.state(cond22)
            states23 = obj.Decisions.States.state(cond23)
            states24 = obj.Decisions.States.state(cond24)
            states25 = obj.Decisions.States.state(cond25)
            states26 = obj.Decisions.States.state(cond26)
            states27 = obj.Decisions.States.state(cond27)
            states28 = obj.Decisions.States.state(cond28)
            statesnogps = obj.Decisions.States.state(condnogps)
            statesonlygps = obj.Decisions.States.state(condonlygps)

            states1to5 = obj.Decisions.States.state(cond1to5)
            states6to10 = obj.Decisions.States.state(cond6to10)
            states11to15 = obj.Decisions.States.state(cond11to15)
            states16to20 = obj.Decisions.States.state(cond16to20)
            states21to25 = obj.Decisions.States.state(cond21to25)
            states26to30 = obj.Decisions.States.state(cond26to30)
            states1to15 = obj.Decisions.States.state(cond1to15)
            states16to30 = obj.Decisions.States.state(cond16to30)
        end
    else
        flag = 0;
    end
end


