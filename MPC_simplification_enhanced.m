function [Hdb, Fdbt] = MPC_simplification_enhanced(Ad, Bd, Cd, Dd, Hz, Q, S, R)
%% Enhanced MPC matrix generation with better numerical conditioning

% Augment system
A_aug = [Ad, Bd; zeros(size(Bd,2), size(Ad,1)), eye(size(Bd,2))];
B_aug = [Bd; eye(size(Bd,2))];
C_aug = [Cd, zeros(size(Cd,1), size(Bd,2))];

% Add small regularization for numerical stability
reg = 1e-6 * eye(size(A_aug));

% Prepare weight matrices
CQC = C_aug' * Q * C_aug;
CSC = C_aug' * S * C_aug;
QC = Q * C_aug;
SC = S * C_aug;

% Initialize block matrices
n_x = size(CQC,1);
n_u = size(R,1);
n_aug = size(A_aug,1);

Qdb = zeros(n_x*Hz, n_x*Hz);
Tdb = zeros(size(QC,1)*Hz, size(QC,2)*Hz);
Rdb = zeros(n_u*Hz, n_u*Hz);
Cdb = zeros(n_aug*Hz, n_u*Hz);
Adc = zeros(n_aug*Hz, n_aug);

%% Build block matrices
for i = 1:Hz
    % Q and T matrices
    if i == Hz
        Qdb(1+n_x*(i-1):n_x*i, 1+n_x*(i-1):n_x*i) = CSC + reg;
        Tdb(1+size(QC,1)*(i-1):size(QC,1)*i, ...
            1+size(QC,2)*(i-1):size(QC,2)*i) = SC;
    else
        Qdb(1+n_x*(i-1):n_x*i, 1+n_x*(i-1):n_x*i) = CQC + reg;
        Tdb(1+size(QC,1)*(i-1):size(QC,1)*i, ...
            1+size(QC,2)*(i-1):size(QC,2)*i) = QC;
    end
    
    % R matrix
    Rdb(1+n_u*(i-1):n_u*i, 1+n_u*(i-1):n_u*i) = R;
    
    % C matrix for predictions
    for j = 1:Hz
        if j <= i
            Cdb(1+n_aug*(i-1):n_aug*i, 1+n_u*(j-1):n_u*j) = ...
                A_aug^(i-j) * B_aug;
        end
    end
    
    % A matrix for predictions
    Adc(1+n_aug*(i-1):n_aug*i, :) = A_aug^i;
end

%% Final matrices with regularization for positive definiteness
Hdb = Cdb' * Qdb * Cdb + Rdb + 1e-6 * eye(size(Rdb));
Fdbt = [Adc' * Qdb * Cdb; -Tdb * Cdb];

end