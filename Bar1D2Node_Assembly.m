% Bar1D2Node_Assembly.m


function z = Bar1D2Node_Assembly(KK, k, i, j)
    % Assemble global stiffness using local element stiffness
    % 
    % Args:
    %       k: element stiffness
    %       i, j: node numbering of the element
    % 
    % Returns:
    %       global stiffness matrix KK
    DOF(1) = i;
    DOF(2) = j;
    for n1 = 1:2
        for n2 = 1:2
            KK(DOF(n1), DOF(n2)) = KK(DOF(n1), DOF(n2)) + k(n1, n2);
        end
    end
    z = kk;
    