function out = proj(THETA, Y, THETAMAX, TOL)
% projection operator
% theta, y: (column) vectors in R^n
% thetamax: ||theta|| <= thetamax
% tol: convergence tolerance

m = size(THETA, 2);
out = zeros(size(THETA, 1), m);
for col = 1:m
    theta = THETA(:, col);
    y = Y(:, col);
    thetamax = THETAMAX(col);
    tol = TOL(col);

    f = (theta'*theta - thetamax^2)/(tol*thetamax^2);
    grad_f = 2*theta/(tol*thetamax^2);
    grad_f_y = grad_f' * y;
    
    if f>=0 && grad_f_y>0
        out(:, col) = y - grad_f/(norm(grad_f)^2) * grad_f_y * f;
    else
        out(:, col) = y;
    end
end

end