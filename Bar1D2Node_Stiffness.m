% Bar1D2Node_stiffness.m


function k = Bar1D2Node_Stiffness(E, A, L)
    % Calculate 2-node element stiffness matrix
    % 
    % Args:
    %       E: Elastic modulus
    %       A: Cross-sectional area
    %       L: Element length
    % 
    % Returns:
    %       Element stiffness k(2x2)
    k = [E*A/L -E*A/L; -E*A/L E*A/L];
    