function [Hdb, Fdbt] = MPC_simplification(Ad, Bd, Cd, Dd, Hz, Q, S, R)
%% Generate matrices for quadprog cost function
% Based on derivations in Section 3.3 and Equation (3.22)

% Augment system to include past inputs as states (Equation 3.14)
A_aug = [Ad, Bd; zeros(size(Bd,2), size(Ad,1)), eye(size(Bd,2))];
B_aug = [Bd; eye(size(Bd,2))];
C_aug = [Cd, zeros(size(Cd,1), size(Bd,2))];

% Prepare weight matrices for the horizon
CQC = C_aug' * Q * C_aug;
CSC = C_aug' * S * C_aug;
QC = Q * C_aug;
SC = S * C_aug;

% Initialize block diagonal matrices
Qdb = zeros(size(CQC,1)*Hz, size(CQC,2)*Hz);
Tdb = zeros(size(QC,1)*Hz, size(QC,2)*Hz);
Rdb = zeros(size(R,1)*Hz, size(R,2)*Hz);
Cdb = zeros(size(B_aug,1)*Hz, size(B_aug,2)*Hz);
Adc = zeros(size(A_aug,1)*Hz, size(A_aug,2));

%% Build block matrices over prediction horizon
for i = 1:Hz
    % Q and T matrices (different for final horizon)
    if i == Hz
        Qdb(1+size(CSC,1)*(i-1):size(CSC,1)*i, ...
            1+size(CSC,2)*(i-1):size(CSC,2)*i) = CSC;
        Tdb(1+size(SC,1)*(i-1):size(SC,1)*i, ...
            1+size(SC,2)*(i-1):size(SC,2)*i) = SC;
    else
        Qdb(1+size(CQC,1)*(i-1):size(CQC,1)*i, ...
            1+size(CQC,2)*(i-1):size(CQC,2)*i) = CQC;
        Tdb(1+size(QC,1)*(i-1):size(QC,1)*i, ...
            1+size(QC,2)*(i-1):size(QC,2)*i) = QC;
    end
    
    % R matrix (input weights)
    Rdb(1+size(R,1)*(i-1):size(R,1)*i, ...
        1+size(R,2)*(i-1):size(R,2)*i) = R;
    
    % C matrix for predictions (Equation 3.20)
    for j = 1:Hz
        if j <= i
            Cdb(1+size(B_aug,1)*(i-1):size(B_aug,1)*i, ...
                1+size(B_aug,2)*(j-1):size(B_aug,2)*j) = A_aug^(i-j) * B_aug;
        end
    end
    
    % A matrix for predictions (Equation 3.20)
    Adc(1+size(A_aug,1)*(i-1):size(A_aug,1)*i, 1:size(A_aug,2)) = A_aug^i;
end

%% Final matrices for quadprog (Equation 3.22)
Hdb = Cdb' * Qdb * Cdb + Rdb;
Fdbt = [Adc' * Qdb * Cdb; -Tdb * Cdb];

end