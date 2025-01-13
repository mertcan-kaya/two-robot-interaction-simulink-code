function tau_i = prcNE(p,q,qd,qdd,g)

    n = 6;

    omegai_h = zeros(3,1,n);
    omegah_h = zeros(3,1,n+1);
    alphah_h = zeros(3,1,n+1);
    Uh_h = zeros(3,3,n+1);
    ah_h = zeros(3,1,n+1);
    Fi_i = zeros(3,1,n);
    Ni_i = zeros(3,1,n);

    fi_i = zeros(3,1,n);
    fh_i = zeros(3,1,n+1);
    ni_i = zeros(3,1,n+1);
    tau_i = zeros(n,1);

    % Forward recursion (link 1 to n)
    zi_i = [0;0;1];
    ah_h(:,:,1) = -g;
    for i = 1:n
        omegai_h(:,:,i) = q.Rh_i(:,:,i)'*omegah_h(:,:,i);
        omegah_h(:,:,i+1) = omegai_h(:,:,i) + qd(i)*zi_i;
        alphah_h(:,:,i+1) = q.Rh_i(:,:,i)'*alphah_h(:,:,i) + qdd(i)*zi_i + cross(omegai_h(:,:,i), qd(i)*zi_i);
        ah_h(:,:,i+1) = q.Rh_i(:,:,i)'*(ah_h(:,:,i) + Uh_h(:,:,i)*p.rh_h_i(:,:,i));

        Uh_h(:,:,i+1) = SkewSym(alphah_h(:,:,i+1)) ...
            + SkewSym(omegah_h(:,:,i+1))*SkewSym(omegah_h(:,:,i+1));

        Fi_i(:,:,i) = p.m_i(i)*ah_h(:,:,i+1) + Uh_h(:,:,i+1)*p.ci_i(:,:,i);
        Ni_i(:,:,i) = p.Ii_i(:,:,i)*alphah_h(:,:,i+1) ...
            + cross(omegah_h(:,:,i+1), p.Ii_i(:,:,i)*omegah_h(:,:,i+1)) ...
            + cross(p.ci_i(:,:,i), ah_h(:,:,i+1));
    end

    % Backward recursion (link n to 1)
    for i = n:-1:1
        fi_i(:,:,i) = Fi_i(:,:,i) + fh_i(:,:,i+1);
        fh_i(:,:,i) = q.Rh_i(:,:,i)*fi_i(:,:,i);
        ni_i(:,:,i) = Ni_i(:,:,i) + q.Rh_i(:,:,i+1)*ni_i(:,:,i+1) ...
            + cross(p.rh_h_i(:,:,i+1), fh_i(:,:,i+1));

        tau_i(i) = ni_i(:,:,i)'*zi_i;
    end

    function S = SkewSym(r)
        S = [0, -r(3), r(2)
            r(3), 0, -r(1)
            -r(2), r(1), 0];
    end

end